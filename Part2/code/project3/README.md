# ENPM661 - Project 3
Tyler Barrett - tbarret2

Mahima Arora - marora1



## Dependencies

The only two dependencies needed for this software are NumPy and OpenCV. 
Feel free to install either of these using `pip` to properly configure the environment.

    pip install numpy
    pip install opencv-python

## Running in Gazebo

Copy the folder "project3" in your catkin_ws folder.

In order to run this in Gazebo, simply run these two commands:

1.   roslaunch project3 map.launch

This will start Gazebo environment with the Map.

2.   rosrun project3 main.py

This command will start the script and prompt user to enter the input.

# Changes before running file if required
To get the expected results, please change the variable STEP_SIZE in main.py before running. 

- Testcase 1 - STEP_SIZE = 30

- Testcase 2 - STEP_SIZE = 10

## Input

This software accepts the start as 3 comma-delimited integers and goal poses as 2 comma-delimited integers.
start - These should be entered as the x coordinate, y coordinate, and orientation angle (theta) in degrees.
When prompted, simply type that in and press Enter. 
The following are acceptable inputs: (what you will see on the terminal)
NOTE: Please note that these inputs are based on GAZEBO COORDINATES as mentioned in the file provided to us. The GAZEBO COORDINATES are different than the 2D VISUALIZATION.


# Testcase 1
Please input the starting point and orientation: -4,-4,30        # (1,1,30) in 2D VIZ 
Please input the goal point: 4,4            # (9,9) in 2D VIZ 
Please enter the desired rpms: 2,3
Finally, please enter the desired clearance: 0.02

# Testcase 2
Please input the starting point and orientation: -4,-4,30    # (1,1,30) in 2D VIZ 
Please input the goal point: -2,1 # (3,6) in 2D VIZ 
Please enter the desired rpms: 2,3 
Finally, please enter the desired clearance: 0.02


## Output
 
 The results for each testcase is given in the output folder in zip.

# Testcase 1 - gazebo_44testcase.mp4

# Testcase 2 - gazebo_21testcase.mp4
