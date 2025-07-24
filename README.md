# fast_lio_nav2_humanoid

Localization and navigation stack for the Unitree H1-2 using fast lio and Nav2.

## Usage

1. Clone this repository onto the robots pc into a folder

2. Build and run the docker container

## To move the robot

python walking_controller.py

make sure the robot is under strict supervision. Unpredictable behavior like walking into objects can happen.

## Visualize

We recommend opening rviz not on the robots pc due to low frames.

## Acknowledgments
This work is based on [https://github.com/hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO) and uses the modified livox_ros_driver2 from [FAST_LIO_LOCALIZATION_HUMANOID](https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID)
