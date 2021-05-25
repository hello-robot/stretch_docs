# Stretch Body: Parameters Tutorial

In this tutorial we will discuss how parameters are managed in Stretch Body and show examples of how to customize your robot by overriding parameters.

## Overview

Stretch Body is organized as a set of classes that extend the base [Device]() class. Each Device has access to a set of parameters that are stored within the Device as a dictionary. 

For example, let's look at the Arm class:

```python
class Arm(Device):
    def __init__(self):
        Device.__init__(self,'arm')
        ...
```

When instantiating its Device base class it loads the robot parameters a various YAML files and Python dictionarys

```python
class Device:
    def __init__(self,name):
        self.name=name
        self.user_params, self.robot_params = RobotParams.get_params()
        try:
            self.params=self.robot_params[self.name]
        except KeyError:
            print('No device params found for %s'%name)
            self.params={}
```

We can explore these parameters via iPython. The Arm has a `params` dictionary which are the parameters that are specific to the Arm. 

```python
In [1]: from stretch_body.arm import Arm
In [2]: a=Arm()
In [3]: a.params
Out[3]: 
{'chain_pitch': 0.0167,
 'chain_sprocket_teeth': 10,
 ...
 'range_m': [0.0, 0.5202755326289126],
 'verbose': 0}

```

It also has a `robot_params` dictionary which are the entire set of parameters for the [Robot]() device. Each Device that is a part of the Robot is a key in `robot_params`

```python
In [5]: a.robot_params.keys()
Out[5]: 
['arm',
 'lift'
 'head',
 'wrist_yaw',
 ...]


```

You can set any of the `robot_params` pro grammatically. For example to adjust the contact sensitivity for the arm:

```python
In [9]: a.params['contact_thresh_N']
Out[9]: [-64.46241590881348, 66.51084520568847]
    
In [10]: a.params['contact_thresh_N']=[-80.0, 80.0]
```

