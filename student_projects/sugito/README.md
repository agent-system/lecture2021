# Sugito README

## How to compile and run
- Open directory in webots
- Open `webots/controllers/khr-2hv_demo/khr2.cpp` and `webots/controllers/mavic2pro/mavic2pro.c`, compile them by pushing Gear button
- Open `webots/worlds/mavic_2_pro.wbt`, run the world
- The robot will automatically take off. After the landing, khr can be operated manually.

## How it works
- KHR2 and Mavic2 are connected directly.
- Mavic2 takes off, searches for a green apple. When detected by recognition module, the robot approaches it in position and height. Finally, it lands in front of the apple.
