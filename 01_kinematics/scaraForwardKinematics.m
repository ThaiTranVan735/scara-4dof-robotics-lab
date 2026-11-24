function [T04, joint_positions] = scaraForwardKinematics(q, params)
% scaraForwardKinematics Giai dong hoc thuan tay may SCARA 4-DOF (RRPR)
theta1 = q(1);
theta2 = q(2);
d3     = q(3);
theta4 = q(4);

T01 = dhTransform(params.L1, 0,  params.d1, theta1);
T12 = dhTransform(params.L2, pi, 0,         theta2);
T23 = dhTransform(0,         0,  d3,        0);
T34 = dhTransform(0,         0,  params.d4, theta4);

T02 = T01 * T12;
T03 = T02 * T23;
T04 = T03 * T34;

p0 = [0; 0; 0];
p1 = T01(1:3, 4);
p2 = T02(1:3, 4);
p3 = T03(1:3, 4);
p4 = T04(1:3, 4);

joint_positions = [p0, p1, p2, p3, p4];
end