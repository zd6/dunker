# Dunker
ECE470 robot simulation group project

Jian Chen, Zhengguan Dai, Zhehong Wang

The project is trying to mock the dunking motion in basketball using a combination of robot car and robot arm. It uses inverse kinematics to calculate the joint angle so it can pick up the ball or place the ball into the basket. It also uses GPS data to orient and position the robot car to the correct position, force-sensor to detect if it successfully picked up the ball. The results show it can successfully find both the ball and the basket in almost all placements. In the project, we gained insight on the Coppelia simulator. The robot is still crude now, as we used lots of features that’s unrealistic like the GPS. In the future, we might use more realistic sensors like infrared or supersonic distance sensors to find the ball and the basket, and use path planning in a field with potential obstacles, or we can let the robot throw the ball inside of dunking to increase the difficulties.

## System Requirement
To run the python-CoppeliaSim code, we recommend python with version later than 3.6 and CoppeliaSim Edu version 4.0.
The python code requires SciPy and NumPy libraries. The CoppeliaSim must have python API option selected.
## Code Instruction
Open CoppeliaSim Edu(ver 4.0) and open scence file 'dunker.ttt', then run the python code 'go_dunker' to see the demo.
