# Robot Care Guide
This guide describes best practices to maintain a happy and healthy Stretch RE1. 
## Battery Maintenance

The robot has two deep-cycle sealed lead acid batteries which provide 12V/19AH of charge. The provided charger will keep the system at a near full-charge if it is left plugged during non-tethered use. 

While deep-cycle batteries are robust to deep discharge states, it is recommended to avoid doing so if possible. The provide charger is able to recover form such a state if it occurs however.

We recommend leaving the charger plugged in whenever the robot base is stationary.

#### Battery State

The battery charger LEDs provide an approximate indicator of battery charge. However, this is not especially accurate. A charged battery will typically report a voltage of 12-12.8V and will maintain that voltage across load conditions. Meanwhile, a partially charged battery may report anywhere from 10-12.8V but its voltage will drop rapidly when loaded. 

The  voltage and current draw can be checked by;

```bash
>>$ stretch_robot_battery_check.py 
[Pass] Voltage with 12.9889035225
[Pass] Current with 2.46239192784
[Pass] CPU Temp with 56.0

```

#### Run Time

The run time for a fully charged system is dependent on the load use case. The vast majority of battery power is consumed by the NUC computer. While the motors can momentarily draw surge currents, the robot is designed such that minimal power is expended to hold the body up against gravity (which keeps average motor current low). 

The robot will continue to operate at battery voltages as low as 9.5V. A fully charged robot running a very high CPU load will take over 2 hours to discharge to 10V. 

#### Low Voltage Alert

When the battery voltage drops below a YAML specified threshold the robot will produce an intermittent double beep sound. This is a reminder to the user to plug in the charger. This threshold is set at 10.5V at the factory but may be overridden.

### Battery Fuse

The batteries are fused with a 20mm 8A / slow blow fuse. Nominal load currents for the robot are 3-5A for most use cases. The fuse provides protection against an internal short. 

Should the fuse blow, the robot will fail to power up. Should the fuse need replacement contact support@hello-robot.com.

## Belt Tension

A neoprene timing belt drives the arm up and down the lift. It may detension over long periods of time if it experiences sustained loading. In this case, slack will become visually apparent in the belt as the lift moves.

The belt is very straightforward to re-tension. Please contact support@hello-robot.com for tensioning instructions.

## Keeping the Robot Clean

The robot surfaces can be wiped down with an alcohol wipe or a moist rag from time to time in order to remove and debris or oils that accumulate on the shells or mast. 

The drive wheels can accumulate dust over time and begin to lose traction. They should be periodically wiped down as well.

When possible, the Trunk cover for the base should be kept on in order to keep dust and debris out of the Trunk connectors.

If the D435i camera requires cleaning use appropriate lens cleaning fluid and a microfiber cloth.

## Keeping the Robot Calibrated

The robot comes pre-calibrated with a robot-specific URDF. This calibration allows the D435i depth sensor to accurately estimate where the robot wrist, and body, is in the depth image.

The robot may become slightly uncalibrated over time for a variety of reasons:

* Normal wear and tear and loosening of joints of the robot
* The head structure is accidentally load and the structure becomes very slightly bent
* The wrist and should structure become accidentally highly loaded and become slightly bent

The calibration accuracy can be checked using the provided ROS tools. If necessary, the user can recalibrate the robot. See the [Stretch URDF Calibration Guide](https://github.com/hello-robot/stretch_ros/blob/master/stretch_calibration/README.md) for more information.

## Transporting the Robot

Stretch was designed to be easily transported in the back of a car, up a stair case, or around a building.

For short trips, the robot can be simply rolled around by grabbing its mast. It may be picked up by its mast and carried up stairs as well. 

**For safety, please use two people to lift the robot.**

For longer trips it is recommended to transport the robot in its original cardboard box with foam packaging. The metal protective cage that surrounds the head is only necessary if the robot might be shipped and the box will not remain upright.

## System Check

It is useful to periodically run stretch_robot_system_check.py. This  will check that the robot's hardware devices are  present and within normal operating conditions. 

```bash
>>$ stretch_robot_system_check.py

---- Checking Devices ----
[Pass] : hello-wacc
[Pass] : hello-motor-left-wheel
[Pass] : hello-motor-arm
[Pass] : hello-dynamixel-wrist
[Pass] : hello-motor-right-wheel
[Pass] : hello-motor-lift
[Pass] : hello-pimu
[Pass] : hello-respeaker
[Pass] : hello-lrf
[Pass] : hello-dynamixel-head

---- Checking Pimu ----
[Pass] Voltage = 12.8763639927
[Pass] Current = 3.25908634593
[Pass] Temperature = 36.3404559783
[Pass] Cliff-0 = -4.72064208984
[Pass] Cliff-1 = -8.56213378906
[Pass] Cliff-2 = 1.08505249023
[Pass] Cliff-3 = 5.68453979492
[Pass] IMU AZ = -9.80407142639


---- Checking EndOfArm ----
[Dynamixel ID:013] ping Succeeded. Dynamixel model number : 1060
[Pass] Ping of: wrist_yaw
[Pass] Calibrated: wrist_yaw

[Dynamixel ID:014] ping Succeeded. Dynamixel model number : 1060
[Pass] Ping of: stretch_gripper
[Pass] Calibrated: stretch_gripper


---- Checking Head ----
[Dynamixel ID:012] ping Succeeded. Dynamixel model number : 1060
[Pass] Ping of: head_tilt

[Dynamixel ID:011] ping Succeeded. Dynamixel model number : 1060
[Pass] Ping of: head_pan


---- Checking Wacc ----
[Pass] AX = 9.4840593338


---- Checking hello-motor-left-wheel ----
[Pass] Position = 43.9992256165


---- Checking hello-motor-right-wheel ----
[Pass] Position = 15.1164712906


---- Checking hello-motor-arm ----
[Pass] Position = 59.7719421387
[Pass] Position Calibrated = True


---- Checking hello-motor-lift ----
[Pass] Position = 83.7744064331
[Pass] Position Calibrated = True


---- Checking for Intel D435i ----
Bus 002 Device 016: ID 8086:0b3a Intel Corp. 
[Pass] : Device found 

```

------
.<div align="center"> All materials are Copyright 2020 by Hello Robot Inc. The Stretch RE1 robot has patents pending</div>