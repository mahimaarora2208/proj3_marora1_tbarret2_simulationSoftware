# ENPM661 - Project 2
Tyler Barrett - tbarret2

Mahima Arora - marora1

https://github.com/tybarrett/Planning-Project-3-Phase-1

## Dependencies

The only two dependencies needed for this software are NumPy and OpenCV. 
Feel free to install either of these using `pip` to properly configure the environment.

    pip install numpy
    pip install opencv-python

## Running the Software 

In order to run the software, simply navigate to the the `Planning-Project-3-Phase-1` folder and run the following command in the terminal:

    python main.py

Feel free to run `main.py` using a different Python interpreter by replacing `python` with a reference to a new interpreter:

    /path/to/your/python main.py

This software will generate a video output file called `output.avi`. 
You can view this video to see how the explored region expands over time. 
Light blue pixels denote free unexplored areas, dark blue pixels show the free explored coordinates.
Brown regions represent the obstacles in the scene, which have been inflated by 5 pixels. 
At the end of the video, a green line will connect the start location to the end location.

## Input

This software accepts the start and goal poses as 3 comma-delimited integers.
These should be entered as the x coordinate, y coordinate, and orientation angle (theta) in degrees.
When prompted, simply type that in and press Enter. 
The following are acceptable inputs:

    Please input the starting point: 123, 123, 30
    Please input the starting point: 123,123,30
    Please input the starting point:    123  ,   123 , 30

The input for the goal point expects the same format. 

The input for the step size, clearance margin, and robot radius should be single integers.
Any decimals will be truncated to an integer. 

## Visualize Search in Real Time
 
If desired, this code can visualize the search in real time.
This is done by creating a new window using OpenCV's `imshow` method. 
Enable this feature by changing `VISUALIZE = False` to `True` in `visualization.py` on line 18.
This will significantly increase processing time, but can be used to better follow the algorithm if desired. 

