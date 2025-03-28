# rm_calibration_controllers

## Overview

Since the zero point of some motors will change after power off, rm_calibration_controller will move at a certain speed after it is started until motors reach the mechanical limit, and motors position will be reset to zero.

**Keywords:** calibration, ROS, position.

### License

The source code is released under a [ BSD 3-Clause license](https://github.com/rm-controls/rm_controllers/blob/master/rm_calibration_controllers/LICENSE).

**Author: DynamicX<br />
Affiliation: DynamicX<br />
Maintainer: DynamicX**

The package has been tested under [ROS](https://www.ros.org/) Indigo, Melodic and Noetic on respectively Ubuntu 18.04 and 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

### Hardware interface type

+ `EffortJointInterface` Used to send effort command to target joint to make it reach the calibration speed.
+ `ActuatorExtraInterface` Used to obtain the information of the target actuators offset, current position, the state of the whether it is stopped and the state of whether it is calibrated.


## Installation

### Installation from Packages

To install all packages from the this repository as Debian packages use

    sudo apt-get install ros-noetic-calibration-controllers

Or better, use `rosdep`:

	sudo rosdep install --from-paths src

### Dependencies

* roscpp
* rm_common
* effort_controllers

## ROS API

#### Service

* **`is_calibrated`** ([control_msgs/QueryCalibrationState](http://docs.ros.org/en/api/control_msgs/html/srv/QueryCalibrationState.html))

  When requesting to this server, it will return response about whether target joint has been calibrated.


#### Parameters

* **`search_velocity`** (double)

  The joint velocity of calibrating.

* **`threshold`** (double)

  This is velocity `threshold`. When the real time velocity of target joint lower than threshold, and last for a while,
  it can switch CALIBRATED from MOVING.

#### Complete description

```yaml
cover_controller:
  type: effort_controllers/JointPositionController
  joint: "cover_joint"
  pid: { p: 15.0, i: 0.0, d: 1.2, i_clamp_max: 0.0, i_clamp_min: 0.0, antiwindup: true, publish_state: true }
```


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/rm-controls/rm_controllers/issues).
