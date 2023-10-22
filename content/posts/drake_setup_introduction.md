---
title: "Drake setup and introduction to IK using pseudo inverse and quadratic programming"
date: 2023-10-15T18:49:58-05:00
draft: False
tags: ["Drake", "Manipulation", "Robotics", "Introduction"]
---


# Introduction to Drake

Drake is a C++ toolbox started by Russ Tedrake and his group at MIT. 
It is a collection of libraries for efficient rigid body dynamics computations and constraint-based multibody dynamics, mainly used for manipulation planning and control.

# Setup for manipulation

In this project I am borrowing most of the work from the [Drake manipulation repository](https://github.com/RussTedrake/manipulation), which is used to teach the manipulators course at MIT. 
Here are the course documents: [manipulation](http://manipulation.csail.mit.edu/) and [Manipulation youtube playlist from last semester](https://youtube.com/playlist?list=PLkx8KyIQkMfUSDs2hvTWzaq-cxGl8Ha69&si=xwgcVbpWYG-cGHDa).

This document will run the simple pick & place example from the course and also add some of my own functions to switch the solving type.


## Setup for this tutorials are as follows:

1. clone the repository: 
``` bash
git clone https://github.com/Prakyathkantharaju/Hacking-robotics
```

2. Setup python env 
Install python version of drake. 
``` bash
pip install pydrake
```

3. In the robotics directory, clone the manipulation repository, I am not sure if this is also a pip installable package but I prefer to clone the repository as you can edit yaml file to add your own robot. 
``` bash
git clone https://github.com/RussTedrake/manipulation
```
I will be using loading and viusalizations tools from the repository, I will be adding the location of this repository to the python path in the next step.
```python
import sys
sys.path.append('../manipulation')
```

## Running the pick and place example

The full python file is located in `robotics/drake/pick_plane_inverse_qp.py`

I will break the code into sections and explain each section here.

1. Importing the necessary libraries
NOTE: I have the manipulator library in the robotics/drake/manipulation directory, so I am adding that to the python path. If you have installed the manipulation library as a pip package, you can skip this step. If you have cloned the manipulation repository, you can add the path to the python path.
```python
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
```

2. Loading the robot and the environment
Here we are loading the meshcat server, which is a visualization tool for drake.
After that we are loading the scenario, which is a yaml file that has the robot and the environment information.
We use the diagram builder to add the scenario to the diagram.

```python
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
```

3. Setting the default pose for the robot.
Here are getting the plant for the robot and setting the default pose for the elements in the diagram.
```python
    # get the plant
    plant = station.GetSubsystemByName("plant")
    # set the pose for the body
    plant.SetDefaultFreeBodyPose(plant.GetBodyByName("base_link"), 
                                 RigidTransform(RotationMatrix.MakeZRotation(np.pi/2), [0,-0.6,0]))


    # set the pose for the gripper 
    get_context = station.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(get_context)
```

4. Generating the trajectories for the robot.
First we have set the initial and final pose for the robot. 
This is predetermined, I did not change it.
Both in the body frame and the world frame, here is the code for that:
```python
    # making trajectory
    X_G = {"initial": plant.EvalBodyPoseInWorld(plant_context, plant.GetBodyByName("body")),
           "goal": RigidTransform(RotationMatrix.MakeZRotation(np.pi), [0.5, 0, 0.0])} # x with respect to the world frame
    X_O = {"initial": plant.EvalBodyPoseInWorld(plant_context, plant.GetBodyByName("base_link")), # x with respect to the body frame
           "goal": RigidTransform(RotationMatrix.MakeZRotation(np.pi), [0.5, 0, 0.0])} # x with respect to the world frame
```
Now using this start and end point, we make the trajectory for the end effector for the robot here is the code for that.
So this is a big function, which I intend to break down into smaller functions later. But let me go step by step what is happening here.
- First I use the start and end point and get the parts of the trajectory that are predetermined, like the pregrasp, grasp, preplace, place, etc. 
- Then we set the time between the trajectory points, this is also predetermined, I did not change it.
- After this I use the piecewise linear function to interpolate the trajectory between the points.
- Then I differentiate the trajectory to get the velocity trajectory.
- Then I plot the trajectory, this is just for visualization if the `plot` flag is set to `True`.

```python
    # making the trajectory for the end effector
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
```

5. Setting up the gripper trajectory.
Since the gripper is a separate entity, we need to set the trajectory for the gripper separately. Here we are using the a `zero-order-trajectory`, i.e it will be actuated at a distinct time.

All these time are predetermined and follows the trajectory of the end effector.

```python
    # making the gripper trajectory

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
```

6. Add these trajectories to the diagram.
```python
    Position_X, times, vel_trajectory = make_trajectory(X_G, X_O, plot=True)

    # gripper position
    traj_wsg_command = MakeGripperCommandTrajectory(times)
    

    # add the trajectory to the diagram
    V_G_source = builder.AddSystem(TrajectorySource(vel_trajectory))
    V_G_source.set_name("V_G_source")
```

7. Setting up the inverse kinematics controller for the robot.

For this I using two different methods, one is the pseudo inverse and the other is the qp (quadratic programming).
Both controller are leaf systems, which means I can connect to the diagram as a system.
I will explain the pseudo inverse controller first and then I will explain the qp controller.

### Pseudo inverse controller
Initialization in `__init__`:
- Initialize the leaf super class.
- Store the plant context, this will help get the positions and vel of the robot.
- Get the model instance for the robot.
- Get the body frame and the world frame for the robot.
- Get the start and end joints for the robot, this will help get the jacobian for the robot.
- Set the input port of the leafsystem, this is essentially a input port to our system, in this case we are taking the desired vel from trajectory source.
- In addition to the desired vel, we also need the position of the robot, so we are setting another input port for the position of the robot.
- Set the output port of the leafsystem, this is an output port to our system, in this case we are giving the desired vel to the robot.
- We are also connecting the output port to the function which will calculate the actual output. In my case this will be `CalcOutput` function.

CalcOutput function:
- Get the current position of the robot and the desired vel from the input port by using the diagram context.
- I set the plant pose based on the position, then calculate the jacobian of the robot with respoect to the end joint.
- then removing the unwanted joints from the jacobian.
- then I calculate the pseudo inverse of the jacobian and multiply it with the desired vel to get the actual vel.





```python

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
        # input port for the desired velocity
        self.vel_input_port = self.DeclareVectorInputPort("V_WG", 6)
        # input port for the position of the robot
        self.q_word = self.DeclareVectorInputPort("iiwa.position", 7)
        # output port for the desired velocity
        self.DeclareVectorOutputPort("iiwa.velocity", 7, self.CalcOutput)
        # this is optional to store the trajectory data.
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
```


### QP controller

The next is the qp controller, this is a bit more complicated than the pseudo inverse controller.
Initialization in `__init__`:
- Initialization is same as the pseudo inverse controller.
- In addition I also initializing a variable which will store the jacobian and the desired vel.

CalcOutput function:
- Starting part of the function is same as the pseudo inverse controller.
- After I find the jacobian, I solving a quadratic programming problem to find the actual vel, using the drake math program solver `Solve`.
- I am using the `Solve` function to solve the mathmatical problem, which is defined in the `_define_math_problem` function.

_define_math_problem function:
- This function is used to define the mathmatical problem for the qp solver.
- I am using the `MathematicalProgram` class to define the problem.
- I add the continous variable to the problem, which is the actual vel. ( here I am 15 variables, because the jacobian is also 6 * 15, there are some useless joints there, I kept it for fun, you can remove it if you want)
- I add the bounding box constraint to the problem, this is to make sure the vel is within the limits.
- I also calculate the error using the following equation $error = J * v - v_{desired}$
- I add the cost to the problem, which is the error dot product with itself ( this make the error quadratic, which is needed for the qp solver)
- I return the problem to the solve.

```python
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
```

8. Adding the controllers to the diagram.
```python
    # add the controller to the diagram
    controller = builder.AddSystem(PseudoInverseController(plant))
    # controller = builder.AddSystem(QPController(plant))
```

9. Add the integrator to the diagram. Since the controller return the vel, we need to integrate it to get the position.
```python
    # add the integrator to the diagram
    integrator = builder.AddSystem(Integrator(7))
```

10. Make all the connections.
```python
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
```

11. Visualize the diagram using pydot. I love this part of drake a LOT.
```python
    # visualize the diagram
    print("Diagram built")
    pydot.graph_from_dot_data( diagram.GetGraphvizString())[0].write_svg("station_pick_place.svg")
```

12. Simulate the system.

```python

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
```

13. Plot the results.
```python
    plt.plot(np.array(controller.save_data).reshape(-1,7))
    plt.show()
```


## Results
{{ <video src="pick_place_ik.mp4", controls="yes"> }}


