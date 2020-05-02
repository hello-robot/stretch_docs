![image](./images/HelloRobotLogoBar.png)

## Stretch RE1: Getting Started

Welcome to Stretch RE1! This guide will get you started with your new robot.

We'd welcome your feedback on your initial product experience. As you get to know your robot and start working with it, please let us know of any issues, questions, or points of confusion. Just email us at support@hello-robot.com.

To start, we highly recommend you first go through the [User Guide: Robot Safety](./RE1/User_Guide_Robot_Safety.md) and watch our Robot Safety Video (link).

## Unboxing

Please watch our unboxing video (Coming soon)

## Quick Robot Tour

A few items you'll want to know about right away:

### Power

The entire robot powers up and down with the On/Off switch. When powering down, we recommend selecting 'Power Off' from the Ubuntu Desktop prior to hitting the Off switch

The provided battery charger can be plugged and unplugged at any time during operation. We recommend keeping the charger plugged in whenever it isn't running untethered.



<img src="./images/trunk.png" alt="image" style="zoom:50%;" />

### Runstop

The illuminated button on the head is its Runstop. Just tap it, you'll hear a beep and it will start flashing. This will pause motion of the primary robot joints during operation. This can be useful if the robot makes an unsafe motion, or if you just want to free up the robot motors while you roll it around.

To allow motion once again, hold the button down for two seconds. After the beep, motion can resume.

<img src="./images/runstop.png" alt="image" style="zoom:50%;" />

### Safe Handling

Like any robot, it is possible to break Stretch if you're not careful. Use common sense when applying forces to its joints, transporting it, etc. 

**Things that won't hurt the robot**:

* Push the arm back in when its not trying to hold a position
* Raise and lower the arm when its not trying to hold a position
* Roll the base around when its not trying to hold a position
* Pick up and carry Stretch while holding it by the mast and base (two people please)

**Things to be mindful of**:

* Backdriving the head and wrist. They will backdrive but they want to go at their own speed.

**Things that can hurt the robot**: 

* Driving the wrist and gripper into the base. When the arm and wrist are stowed it is possible to collide the two.
* Getting the gripper stuck on something and the  driving the arm, lift, or base. 
* Laying the robot down with it weight on its camera.
* Trying to ride on the robot, fluids, etc. (eg, common sense)

## Hello World: XBox Teleoperation

Stretch comes ready to run out of the box. The Xbox Teleoperation demo will let you quickly test out the robot capabilities by teleoperating it with an Xbox Controller. 

![image](./images/xbox.png)

To start the demo:

* Remove the 'trunk' cover and power on the robot
* Wait for about 45 seconds. You will hear the Ubuntu startup sound, followed by two beeps, once the system boots.
* Hit the Connect button on the controller. The upper two LEDs of the ring will illuminate.
* Hit the Home Robot button. Stretch will go through its calibration routine.
* You're ready to go! A few things to try:
  * Hit the Stow Robot button. The robot will fold up. You can now pull the Fast Base trigger to make Stretch drive faster
  * Try picking up your cellphone from the floor 
  * Try grasping cup from a counter top
  * Try delivering an object to a person
* If you're done, hold down the Shutdown PC button for 2 seconds. This will cause the PC to turn off. You can then power down the robot. Or proceed to the next step...



## Get Plugged In

Now that you've taken Stretch on a test drive, let's get plugged in.

* Remove the 'trunk' cover and power on the robot if its not already on.
* Plug in a mouse, keyboard and HDMI monitor to the robot trunk
* Plug in the battery charger

Log in to the robot. The default user credentials came in the box with the robot. 

Before you can start coding you'll want to first kill off the XBox controller process which runs by default at boot:

```
Foo.
```

While you're at it, disable this autoboot feature. You can always turn it back on later. 

Search for 'Startup' from Ubuntu Activities. Uncheck the box for 'hello_robot_xbox_teleop' 

![image](./images/xbox_off.png)



## Start Coding...

Open up a Terminal. From the command line, first verify that that all of the hardware is present and happy

```
>> stretch_robot_system_check.py
```

You'll see that a items come back read because they haven't yet been calibrated. Home the robot

```
>> stretch_robot_home.py
```

Once the robot has homed, let's write some quick test code:

```
>>ipython
```

Then,

```python
Python 2.7.17 (default, Apr 15 2020, 17:20:14) 
...

import stretch_robot.robot
robot=stretch_robot.robot.Robot()
robot.startup()
robot.stow( )
robot.arm.move_to(0)
robot.arm.move_to(0.5)
robot.lift.move_to(0.5)
robot.stop()

```

## Check out the Robot URDF

