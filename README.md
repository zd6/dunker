# jumpshooter
ECE470 robot simulation group project

Jian Chen, Zhengguan Dai, Zhehong Wang

The simulator this repository code can work on is Cappelia Robotics' simulator CoppeliaSim V4.0.0 rev4, education distribution. The simulator is compatible with Robot Operating System code.

Currently, we are at an early stage of testing what the simulator can do. We implemented a small snippet of 'lua' code that check if the ball has been gripped in 'hand'. If the ball is in 'hand', we track a curve that simulate a shooting the basketball, while return to initial position if not detecting a ball in firm grip.

Also, we are testing about a locomotive that can move the robot arm around in the court. The mobile robot is called Pioneer, also a pre-build robot in CoppeliaSim.

The forward kinematics is calculated in matlabFile/Jacobian.m and a demo code of controlling robot with api interface is in pyFile/v_rep_code.py
