# Introduction

There exist three toy projects for agent system course:
    1. rl_khr2: A reinforcement learning based demo aims to make the 
                17 DOF humanoid robot, khr-2hv, swim in the pool.

    2. rl_salamander: This demo aims to make salamander swim as fast as possible.
                      By using the rl-based controller, it's able toswim faster 
                      than the webots built-in controller.

    3. salamander_trackball: A simple demo to practice the visual segmentation and recognition in webots 


# Environment setting

A. for project 1 and 2:

    1). Prerequisites:
        a. webots 2021a
        b. python >=3.6
        c. cuda >= 11.0
        d. pytorch >=17.1

    2). install stablebaselines3:
        pip install stable-baselines3

    3). set the controller as external:
        a. open the proto and set the controller as external

    4). enable the pycharm's terminal and command line:
        a. set environment variables:
            WEBOTS_HOME: /usr/local/webots
            LD_LIBRARY_PATH: add ${WEBOTS_HOME}/lib/controller
            PYTHONPATH: add ${WEBOTS_HOME}/lib/controller/python3X
        b. include the supervisor file to the pycharm


B. for the project 3:
    only webots 2021a is required.


# Run the demo
project 1 and 2:
    #in command line:
    python3 ppo_controller.py --mode: train or test
                              --total_timesteps: int
    
    PS: more custom parameters will come soon.

    
