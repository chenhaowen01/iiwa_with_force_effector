# iiwa with force effector

A set of ROS packeges for force control experiment on iiwa. It contains following packages:

* force_effector_control: config files for force effector controller.
* force_effector_controllers: some controller implementation under [*ros_control*](http://wiki.ros.org/ros_control "ros_control"). Only a pid controller was implemented now.
* force_effector_description: robot description file ([*urdf file*](http://wiki.ros.org/ros_control)) for force effector, used in simulation.
* force_effector_gazebo: config file for gazebo simulation enviroment.
* force_effector_hw: hardware interface for force effector (developing).
* ft_gazebo_ros_control: simulation hardware interface with ForceTorqueSensorInterface to replace gazebo's default one.
* iiwa7_with_force_effector_description: robot description file containing iiwa7 and force effector, used in moveit. (deprecated)
* iiwa7_with_force_effector_moveit: moveit config files for iiwa7 with force effector. (deprecated)
* test_iiwa: test code.

## Requirements

* ROS (Kinetic Kame on Ubuntu 16.04 or Melodic Morenia on Ubuntu 18.04)

  See [the official install guide](http://www.ros.org/install) to learn how to install ROS. It is recommended to install Kinetic Kame on Ubuntu 16.04.
  
* IIWA STACK (development branch)
  
  See their [wiki](https://github.com/IFL-CAMP/iiwa_stack/wiki "wiki") and [issues](https://github.com/IFL-CAMP/iiwa_stack/issues) for more infomation. In following sections, we will introduce how to setup in ROS side.

  > note: we used **development branch** of IIWA_STACK.

* canopen (A Python implementation of the CANopen standard.)
  See their [PyPi homepage](https://pypi.org/project/canopen/) and [documentation](http://canopen.readthedocs.io/en/stable/).

* [socketCAN driver for IXXAT USB-to-CAN V2](https://www.ixxat.com/support/file-and-documents-download/drivers/socketcan-driver) or any alternatives.

## Installing

### ROSCORE side (PC running ROS)

1. Create workspace
   ```bash
   mkdir -p iiwa_ws/src && cd iiwa_ws
   catkin_init_workspace
   ```

2. Clone repository
   ```bash
   git clone -b development https://github.com/chenhaowen01/iiwa_stack.git src/iiwa_stack
   git clone https://github.com/UTNuclearRoboticsPublic/netft_utils.git src/netft_utils
   git clone https://github.com/chenhaowen01/iiwa_with_force_effector.git src/iiwa_with_force_effector
   ```

3. Install dependences
   1. ROS dependences
      ```bash
      rosdep install --from-paths src --ignore-src -r -y
      ```
   2. Python dependences
      ```bash
      sudo apt install python-pip
      pip install --user canopen
      ```
   3. CANOpen driver
   
      Download SocketCAN driver from [Ixxat website](https://www.ixxat.com/support/file-and-documents-download/drivers/socketcan-driver), and follow their README to compile and install the dirver.

      > note: IXXAT USB-to-CAN V2 is used in this project currently, but any CAN interfaces whitch provide a socketCAN driver is an alternative.

      > note: When you update your Linux kernel, you may need to recompile and reinstall the socketCAN driver.
      

4. Building
   ```bash
   catkin build
   ```

5. Source workspace
   ```bash
   source devel/setup.bash
   ```

### SUNRISE side (iiwa's controller)
> note: in order to synchronise project between iiwa's controller and your PC, you need to install *Sunrise Workbench* first.

1. Synchornise project between your PC and iiwa's controller with Workbench
2. Download iiwa stack's code from [github](https://github.com/IFL-CAMP/iiwa_stack), you can also use `git clone`.
   > note: we need code from **development branch**.


## Running

### Setup

## Contact

Haowen Chen: chenhaowen01@qq.com