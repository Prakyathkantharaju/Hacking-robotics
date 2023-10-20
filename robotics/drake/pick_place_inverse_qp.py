
# loading the python libraries
import numpy as np
import matplotlib.pyplot as plt
import pydot

# loading the drake variables
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    AngleAxis,
    Context,
    DiagramBuilder,
    Integrator,
    JacobianWrtVariable,
    LeafSystem,
    MeshcatVisualizer,
    MultibodyPlant,
    MultibodyPositionToGeometryPose,
    Parser,
    PiecewisePolynomial,
    PiecewisePose,
    Quaternion,
    Rgba,
    RigidTransform,
    RotationMatrix,
    SceneGraph,
    Simulator,
    StartMeshcat,
    SystemOutput,
    TrajectorySource,
    MathematicalProgram,
    Solve,
)

import sys
sys.path.append('manipulation/')

from manipulation.scenarios import AddMultibodyPlantSceneGraph
from manipulation.station import MakeHardwareStation, load_scenario




def make_trajectory(X_WG: dict, X_WO: dict, plot:bool =True):
    """
    Takes a partial specification with X_G["initial"] and X_O["initial"] and
    X_0["goal"], and returns a X_G and times with all of the pick and place
    frames populated.
    """
    X_G = { "initial": RigidTransform(RotationMatrix.MakeXRotation(-np.pi / 2.0), [0, -0.25, 0.25]) }

    # TODO(prakyath) THIS IS COPIED FROM THE PICK NOTEBOOK, BUT YOU'LL NEED TO MODIFY IT FOR WITH RL LATER.
    # Define (again) the gripper pose relative to the object when in grasp.
    p_GgraspO = [0, 0.12, 0]
    R_GgraspO = RotationMatrix.MakeXRotation(
        np.pi / 2.0
    ) @ RotationMatrix.MakeZRotation(np.pi / 2.0)
    X_GgraspO = RigidTransform(R_GgraspO, p_GgraspO)
    X_OGgrasp = X_GgraspO.inverse()
    # pregrasp is negative y in the gripper frame (see the figure!).
    X_GgraspGpregrasp = RigidTransform([0, -0.08, 0])

    X_WG["pick"] = X_WO["initial"] @ X_OGgrasp
    X_WG["prepick"] = X_WG["pick"] @ X_GgraspGpregrasp
    X_WG["place"] = X_WO["goal"] @ X_OGgrasp
    X_WG["preplace"] = X_WG["place"] @ X_GgraspGpregrasp

    # I'll interpolate a halfway orientation by converting to axis angle and halving the angle.
    X_GprepickGpreplace = X_WG["prepick"].inverse() @ X_WG["preplace"]
    angle_axis = X_GprepickGpreplace.rotation().ToAngleAxis()
    X_GprepickGclearance = RigidTransform(
        AngleAxis(angle=angle_axis.angle() / 2.0, axis=angle_axis.axis()),
        X_GprepickGpreplace.translation() / 2.0 + np.array([0, -0.3, 0]),
    )
    X_WG["clearance"] = X_WG["prepick"] @ X_GprepickGclearance

    # Now let's set the timing
    times = {"initial": 0}
    X_GinitialGprepick = X_G["initial"].inverse() @ X_WG["prepick"]
    times["prepick"] = times["initial"] + 10.0 * np.linalg.norm(
        X_GinitialGprepick.translation()
    )
    # Allow some time for the gripper to close.
    times["pick_start"] = times["prepick"] + 2.0
    times["pick_end"] = times["pick_start"] + 2.0
    X_WG["pick_start"] = X_WG["pick"]
    X_WG["pick_end"] = X_WG["pick"]
    times["postpick"] = times["pick_end"] + 2.0
    X_WG["postpick"] = X_WG["prepick"]
    time_to_from_clearance = 10.0 * np.linalg.norm(
        X_GprepickGclearance.translation()
    )
    times["clearance"] = times["postpick"] + time_to_from_clearance
    times["preplace"] = times["clearance"] + time_to_from_clearance
    times["place_start"] = times["preplace"] + 2.0
    times["place_end"] = times["place_start"] + 2.0
    X_WG["place_start"] = X_WG["place"]
    X_WG["place_end"] = X_WG["place"]
    times["postplace"] = times["place_end"] + 2.0
    X_WG["postplace"] = X_WG["preplace"]


    # combine all the tiem and tracjectories.
    sample_times = []
    X_WG_traj = []
    for name in [
        "initial",
        "prepick",
        "pick_start",
        "pick_end",
        "postpick",
        "clearance",
        "preplace",
        "place_start",
        "place_end",
        "postplace",
    ]:
        sample_times.append(times[name])
        X_WG_traj.append(X_WG[name])
        print(name, X_WG[name].translation())

    # Do a piecewise linear interpolation.
    traj_position_G = PiecewisePose.MakeLinear(sample_times, X_WG_traj)
    print(traj_position_G)


    # Get the trajectories in velocity though differentiation.
    traj_velocity_G = traj_position_G.MakeDerivative()

    # Now let's plot the trajectory.
    if plot:
        fig, ax = plt.subplots()
        plot_time = traj_velocity_G.get_segment_times()
        plot_V_WG = traj_velocity_G.vector_values(plot_time)
        plt.plot(plot_time, plot_V_WG.T)

        ax.legend()
        plt.show()

    return X_WG, times, traj_velocity_G

def MakeGripperCommandTrajectory(times):
    opened = np.array([0.107])
    closed = np.array([0.0])

    traj_wsg_command = PiecewisePolynomial.FirstOrderHold(
        [times["initial"], times["pick_start"]],
        np.hstack([[opened], [opened]]),
    )
    traj_wsg_command.AppendFirstOrderSegment(times["pick_end"], closed)
    traj_wsg_command.AppendFirstOrderSegment(times["place_start"], closed)
    traj_wsg_command.AppendFirstOrderSegment(times["place_end"], opened)
    traj_wsg_command.AppendFirstOrderSegment(times["postplace"], opened)
    return traj_wsg_command

class PseudoInverseController(LeafSystem):
    def __init__(self, plant: MultibodyPlant):
        super().__init__()

        # store the plant for the controller use in future
        self._plant = plant

        # get the default context for the plant for the differential kinematics
        self._get_plant_context = plant.CreateDefaultContext()

        # get the model instance for the iiwa
        self._iiwa = plant.GetModelInstanceByName("iiwa")

        # these two are for views and transformations
        self._g = plant.GetBodyByName("body").body_frame()
        self._w = plant.world_frame()

        # get the start and end joints
        self._start_joint = plant.GetJointByName("iiwa_joint_1").velocity_start()
        self._end_joint = plant.GetJointByName("iiwa_joint_7").velocity_start()

        # start declaring the inputs nad output ports
        self.vel_input_port = self.DeclareVectorInputPort("V_WG", 6)
        self.q_word = self.DeclareVectorInputPort("iiwa.position", 7)
        self.DeclareVectorOutputPort("iiwa.velocity", 7, self.CalcOutput)
        self.save_data = []

    def CalcOutput(self, context, output):
        # get the current position of the robot
        q = self.q_word.Eval(context)

        # get the current velocity of the robot
        v = self.vel_input_port.Eval(context)

        # set the position to the robot. Still not sure why we need to do this.
        self._plant.SetPositions(self._get_plant_context, self._iiwa, q)

        # get the jacobian of the robot
        J = self._plant.CalcJacobianSpatialVelocity(
            self._get_plant_context, JacobianWrtVariable.kV, self._g, [0, 0, 0], self._w, self._w
        )
        J = J[:, self._start_joint : self._end_joint + 1]
        v = np.linalg.pinv(J).dot(v)
        self.save_data.append(v)
        
        output.SetFromVector(v)




#TODO Need to make a controller super class in the future
class QPController(LeafSystem):
    def __init__(self, plant: MultibodyPlant):
        super().__init__()

        # store the plant for the controller use in future
        self._plant = plant

        # get the default context for the plant for the differential kinematics
        self._get_plant_context = plant.CreateDefaultContext()

        # get the model instance for the iiwa
        self._iiwa = plant.GetModelInstanceByName("iiwa")

        # these two are for views and transformations
        self._g = plant.GetBodyByName("body").body_frame()
        self._w = plant.world_frame()

        # get the start and end joints
        self._start_joint = plant.GetJointByName("iiwa_joint_1").velocity_start()
        self._end_joint = plant.GetJointByName("iiwa_joint_7").velocity_start()

        # start declaring the inputs nad output ports
        self.vel_input_port = self.DeclareVectorInputPort("V_WG", 6)
        self.q_word = self.DeclareVectorInputPort("iiwa.position", 7)
        self.DeclareVectorOutputPort("iiwa.velocity", 7, self.CalcOutput)
        self.save_data = [] 

        # Initialize the J and x with None
        self.J = None
        self.v = None

    def _define_math_problem(self, v_desired: np.ndarray):
        prog = MathematicalProgram()
        v = prog.NewContinuousVariables(15, "v")
        v_max = 0.5
        error = self.J @ v - np.array(v_desired)
        prog.AddBoundingBoxConstraint(-v_max, v_max, v)
        prog.AddCost(error.dot(error))
        return prog



    def CalcOutput(self, context: Context, outputs: SystemOutput) -> None:
        # get the current position of the robot
        q = self.q_word.Eval(context)

        # get the current velocity of the robot
        v = self.vel_input_port.Eval(context)

        # set the position to the robot. Still not sure why we need to do this.
        self._plant.SetPositions(self._get_plant_context, self._iiwa, q)

        # get the jacobian of the robot
        self.J = self._plant.CalcJacobianSpatialVelocity(
            self._get_plant_context, JacobianWrtVariable.kV, self._g, [0, 0, 0], self._w, self._w
        )

        # print(v.shape, v)
        result = Solve(self._define_math_problem(v))
        r = result.GetSolution(v)
        v = result.get_x_val()[:7]
        # print(result.get_x_val()[:7], "This is the x val results")
        # print(r.shape, r)
        # v = np.array([print(dir(c[0])) for c in r])

        self.save_data.append(v)
        # print(v)
        # print("v", v, dir(v), type(v))
        # print(np.array(v), dir(v), type(v))
        outputs.SetFromVector(v)
        

def main(plot: bool = False):

    # start the meshcat server
    meshcat = StartMeshcat()


    # scenario
    scenario = \
    """
    directives:
    - add_directives:
        file: package://manipulation/clutter.dmd.yaml
    - add_model:
        name: foam_brick
        file: package://manipulation/hydro/061_foam_brick.sdf
    model_drivers:
        iiwa: !IiwaDriver
            hand_model_name: wsg
        wsg: !SchunkWsgDriver {}
    """
    # start with some blank diagram
    builder = DiagramBuilder()

    # loading the scenriao using the yaml string above.
    scenario = load_scenario(data=scenario)

    # adding scenario to the diagram, the scenario has two parts, iiwa and the wsg. you can visualize this in the diagram.
    station = builder.AddSystem(MakeHardwareStation(
        scenario=scenario, meshcat=meshcat))

    # get the plant
    plant = station.GetSubsystemByName("plant")
    # print(type(plant))
    # set the pose for the body
    plant.SetDefaultFreeBodyPose(plant.GetBodyByName("base_link"), 
                                 RigidTransform(RotationMatrix.MakeZRotation(np.pi/2), [0,-0.6,0]))


    # set the pose for the gripper 
    get_context = station.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(get_context)

    # making trajectory
    X_G = {"initial": plant.EvalBodyPoseInWorld(plant_context, plant.GetBodyByName("body")),
           "goal": RigidTransform(RotationMatrix.MakeZRotation(np.pi), [0.5, 0, 0.0])} # x with respect to the world frame
    X_O = {"initial": plant.EvalBodyPoseInWorld(plant_context, plant.GetBodyByName("base_link")), # x with respect to the body frame
           "goal": RigidTransform(RotationMatrix.MakeZRotation(np.pi), [0.5, 0, 0.0])} # x with respect to the world frame
    print(X_G["goal"], X_G["initial"])

    Position_X, times, vel_trajectory   = make_trajectory(X_G, X_O, plot=True)

    # gripper position
    traj_wsg_command = MakeGripperCommandTrajectory(times)
    

    # add the trajectory to the diagram
    V_G_source = builder.AddSystem(TrajectorySource(vel_trajectory))
    V_G_source.set_name("V_G_source")

    # add the controller to the diagram
    # controller = builder.AddSystem(PseudoInverseController(plant))
    controller = builder.AddSystem(QPController(plant))


    integrator = builder.AddSystem(Integrator(7))
    integrator.set_name("integrator")

    # connections between the systems

    # 1. connect the desired velocity to the controller
    builder.Connect(V_G_source.get_output_port(), controller.GetInputPort("V_WG"))

    # 2. connect the output of the controller to the integrator
    builder.Connect(controller.get_output_port(), integrator.get_input_port())

    # 3. connect the integrator to the iiwa position
    builder.Connect(integrator.get_output_port(), station.GetInputPort("iiwa.position"))

    # 4. connect the iiwa state to the controller
    builder.Connect(station.GetOutputPort("iiwa.position_measured"), controller.GetInputPort("iiwa.position"))

    # calcualte connecting the gripper
    wsg_source = builder.AddSystem(TrajectorySource(traj_wsg_command))
    wsg_source.set_name("wsg.command")

    # 5. gripper position to the station
    builder.Connect(wsg_source.get_output_port(), station.GetInputPort("wsg.position"))

    diagram = builder.Build()
    diagram.set_name("Testing_pick_and_place")

    # visualize the diagram
    print("Diagram built")
    pydot.graph_from_dot_data( diagram.GetGraphvizString())[0].write_svg("station_pick_place.svg")

    # simulate the diagram
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()
    station_context = station.GetMyContextFromRoot(context)

    integrator.set_integral_value(integrator.GetMyContextFromRoot(context),
                                    plant.GetPositions(
                                        plant.GetMyContextFromRoot(context),
                                        plant.GetModelInstanceByName("iiwa"),
                                    ),
    )

    diagram.ForcedPublish(context)
    meshcat.StartRecording(set_visualizations_while_recording=True)
    simulator.AdvanceTo(vel_trajectory.end_time())
    meshcat.StopRecording()
    meshcat.PublishRecording()
    plt.plot(np.array(controller.save_data).reshape(-1,7))
    plt.show()



    


if __name__ == "__main__":
    main()




