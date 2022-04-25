'''
Python script to avoid obstacles in gazebo.

author : Mahima Arora
         Tyler Barrett

Model Specifications: https://emanual.robotis.com/docs/en/platform/turtlebot3/features/      

Model : WAFFLE
- Wheel Diameter = 66mm (r = 33mm)
- Robot Width = 306mm  
- Distance Between the Wheels (L) = 287mm
'''

# USER INPUT
x_start, y_start, start_orientation = input("Enter starting coordinates and orientation e.g. --> 20 20 30 : ").split()
x_goal, y_goal =  input("Enter the x and y coordinates of the goal e.g. --> 50 50 : ").split()
RPM_L, RPM_R = int(input("Enter the Left Wheel RPM and Right Wheel RPM respectively : ")).split()
RPM_L = int(RPM_L)
RPM_R = int(RPM_R)
clearance = int(input("Enter the clearance : "))
step_size = 1
radius = 1
start = (int(x_start) + radius + clearance, int(y_start) + radius + clearance)
goal = (int(x_goal) - radius - clearance, int(y_goal) - radius - clearance)

# TURN_RIGHT, 
ACTION_SET = ([0, RPM_L],[RPM_L, 0],[RPM_L, RPM_L],[0, RPM_R],[RPM_R, 0],[RPM_R, RPM_R],[RPM_L, RPM_R],[RPM_R, RPM_L])
