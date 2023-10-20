
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

from manipulation.scenarios import AddMultibodyPlantSceneGraph #type: ignore
from manipulation.station import MakeHardwareStation, load_scenario #type: ignore


# local imports
from create_trajectory import generate_rotation_and_translations


def circle_curve(initial_tranformation: RigidTransform) -> (dict, dict):
    circle_points = np.array([[np.sin(theta), np.cos(theta), 0] for theta in np.arange(0, 2*np.pi, 0.1)])

    # get the quaternions and translations
    qt_pairs = generate_rotation_and_translations(circle_points)
    times = {}
    X = {}
    p = {}
    for i, qt_pair in enumerate(qt_pairs):
        quaternion, translation = qt_pair
        X[i] = initial_tranformation @ RigidTransform(RotationMatrix(quaternion), p=translation)
        p[i] = translation
        times[i] = i
    return X, p, times

    

def line_curve(initial_tranformation: RigidTransform) -> (dict, dict):
    Z_mid_1 = -0.2
    Z_mid_2 = 0.1
    Z_end = 0.2
    p = {}
    p["initial"] = initial_tranformation.translation()
    p["mid_1"] = [Z_mid_1, 0, 0.1]
    p["mid_2"] = [Z_mid_2, 0, -0.1]
    p["end"] = [Z_end, 0, 0]

    # Not get the transformation matrix
    X = {}
    X["initial"] = initial_tranformation
    X["mid_1"] = X["initial"] @ RigidTransform(RotationMatrix(), p["mid_1"])
    X["mid_2"] = X["mid_1"] @ RigidTransform(RotationMatrix(), p["mid_2"])
    X["end"] = X["mid_2"] @ RigidTransform(RotationMatrix(), p["end"])

    return X , p
    
def MakeGripperCommandTrajectory():
    opened = np.array([0.0])
    closed = np.array([0.0])

    traj_wsg_command = PiecewisePolynomial.FirstOrderHold(
        [0, 1],
        np.hstack([[opened], [opened]]),
    )
    traj_wsg_command.AppendFirstOrderSegment(2, closed)
    traj_wsg_command.AppendFirstOrderSegment(10, closed)
    traj_wsg_command.AppendFirstOrderSegment(12, opened)
    traj_wsg_command.AppendFirstOrderSegment(15, opened)
    return traj_wsg_command

# trajectories
def make_curves(Initial_start_point_Transformation, type: str = "circle"):
    if type == "line":
        # tiems in the increasing order or 5s
        X, p = line_curve(Initial_start_point_Transformation)
        times = [0, 5, 10, 15]
        X_list = [X["initial"], X["mid_1"], X["mid_2"], X["end"]]
    if type == "circle":
        X, p, times = circle_curve(Initial_start_point_Transformation)
        X_list = [X[i] for i in range(len(X))]
        times = [times[i] for i in range(len(times))]

    
    

    # Generate the piecewise PiecewisePose Make linear
    traj_pos_G = PiecewisePose.MakeLinear(times, X_list)

    # Plot the trajectory
    fig, ax = plt.subplots(1, 1, figsize=(6, 6))
    ax.plot([X.translation()[0] for X in X_list], 'o-', c='r')
    ax.plot([X.translation()[1] for X in X_list], 'o-', c='g')
    ax.plot([X.translation()[2] for X in X_list], 'o-', c='b')

    plt.show()

    traj_vel_G = traj_pos_G.MakeDerivative()
    return traj_pos_G, traj_vel_G


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






def main():
    # start the meshcat server
    meshcat = StartMeshcat()


    # scenario
    scenario = \
    """
    directives:
    - add_directives:
        file: package://manipulation/clutter.dmd.yaml
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
    # plant.SetDefaultFreeBodyPose(plant.GetBodyByName("base_link"), 
    #                              RigidTransform(RotationMatrix.MakeZRotation(np.pi/2), [0,-0.6,0]))


    # set the pose for the gripper 
    get_context = station.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(get_context)

    initial_tranformation =  plant.EvalBodyPoseInWorld(plant_context, plant.GetBodyByName("right_finger"))
    traj_position, traj_vel = make_curves(initial_tranformation)
    traj_wsg_command = MakeGripperCommandTrajectory()


    # add the trajectory to the diagram
    V_G_source = builder.AddSystem(TrajectorySource(traj_vel))
    V_G_source.set_name("V_G_source")

    # add the controller to the diagram
    controller = builder.AddSystem(PseudoInverseController(plant))
    # controller = builder.AddSystem(QPController(plant))


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
    diagram.set_name("Testing trajectory")

    # visualize the diagram
    print("Diagram built")
    pydot.graph_from_dot_data( diagram.GetGraphvizString())[0].write_svg("station_traj_plane.svg")

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
    simulator.AdvanceTo(traj_vel.end_time())
    meshcat.StopRecording()
    meshcat.PublishRecording()

if __name__ == "__main__":
    main()

    
    



