clear
syms theta1 theta2 theta3 theta4 theta5 theta6 real
v = [0 0 0.1384];
connection = [0.0445 0 0.2367];
robot_base = [-0.1562 0.0043 0.2882];
p1 = [0.1562 0.0044 0.3927] - robot_base;
o1 = [0 0 1];
p2 = [-0.2672 0.0022 0.3971] - robot_base;
o2 = [-1 0 0];
p3 = [-0.2672 0.2458 0.3968] - robot_base;
o3 = [-1 0 0];
p4 = [-0.2672 0.4591 0.3967] - robot_base;
o4 = [-1 0 0];
p5 = [-0.2679 0.5433 0.3967] - robot_base;
o5 = [0 1 0];
p6 = [-0.2673 0.5444 0.3967] - robot_base;
o6 = [1 0 0];
s1 = screw(o1, p1);
T1 = expm(screw_matrix(o1, p1)*theta1);
J1 = s1;

s2 = screw(o2, p2);
T2 = expm(screw_matrix(o2,p2)*theta2);
J2 = adj(T1)*s2;

s3 = screw(o3, p3);
J3 = adj(T1*T2)*s3;
T3 = expm(screw_matrix(o3,p3)*theta3);

s4 = screw(o4, p4);
J4 = adj(T1*T2*T3)*s4;
T4 = expm(screw_matrix(o4,p4)*theta4);

s5 = screw(o5, p5);
J5 = adj(T1*T2*T3*T4)*s5;
T5 = expm(screw_matrix(o5,p5)*theta5);

s6 = screw(o6, p6);
J6 = adj(T1*T2*T3*T4*T5)*s6;
T6 = expm(screw_matrix(o6,p6)*theta6);

Js = vpa([J1 J2 J3 J4 J5 J6]);