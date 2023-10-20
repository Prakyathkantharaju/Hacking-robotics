---
title: "Drake setup and introduction"
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

3. In the robotics directory, clone the manipulation repository:
``` bash
git clone https://github.com/RussTedrake/manipulation
```
I will be using loading and viusalizations tools from the repository, I will be adding the location of this repository to the python path in the next step.
```python
import sys
sys.path.append('../manipulation')
```
**NOTE**: If you are path is different, please use the appropriate path.



