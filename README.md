# jumpshooter
ECE470 robot simulation  
The simulator this repository code can work on is Cappelia Robotics' simulator CoppeliaSim V4.0.0 rev4, education distribution. The simulator is compatable with Robot Operating System code.

Currently, we are at an early stage of testing what the simulator can do. We implemented a small snippet of ROS code that check if the ball has been gripped in 'hand'. If the ball is in 'hand', we track a curve that simulate a shooting the basketball, while return to initial position if not detecting a ball in firm grip.
