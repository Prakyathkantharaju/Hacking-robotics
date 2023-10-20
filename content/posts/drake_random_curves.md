---
title: "Drake Spatial curves"
date: 2023-10-15T19:01:14-05:00
draft: true
tags: ["Drake", "Manipulation", "Robotics", "Inverse Kinematics"]
---

# Drake Spatial curves

The main aim of this is to setup the manipulation and test the inverse kinematics options in the drake ecosystem.
So lets get started.

## Setup
Follow the [Introduction post from my blog]({{<relref "drake_setup_introduction/#setup-for-manipulation">}}) 

### Loading python packages
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


## Spatial curves
For this example, I am doing very basic curves. First a line, then a circle, I have give code for a a helix as well, but I have not tested that part of the code.


### Line



