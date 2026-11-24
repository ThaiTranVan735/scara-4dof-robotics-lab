clear; clc; close all;

%% 1. Khai bao thong so kich thuoc tay may SCARA 4-DOF (don vi: met)
params.L1 = 0.35;   % Chieu dai canh tay 1
params.L2 = 0.25;   % Chieu dai canh tay 2
params.d1 = 0.30;   % Chieu cao tru de
params.d4 = 0.08;   % Chieu dai truc gan dau cong tac

% Cau hinh khop thu nghiem: [theta1 (rad), theta2 (rad), d3 (m), theta4 (rad)]
q = [pi/4, -pi/3, 0.12, pi/6];

%% 2. Chay ham dong hoc thuan tu viet tu nguyen ly goc
[T_custom, pts] = scaraForwardKinematics(q, params);

%% 3. Dung mo hinh chuan bang Robotics System Toolbox de doi chieu
robot = rigidBodyTree('DataFormat', 'row');

body1 = rigidBody('link1');
jnt1 = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(jnt1, [params.L1, 0, params.d1, 0], 'dh');
body1.Joint = jnt1;
addBody(robot, body1, 'base');

body2 = rigidBody('link2');
jnt2 = rigidBodyJoint('joint2', 'revolute');
setFixedTransform(jnt2, [params.L2, pi, 0, 0], 'dh');
body2.Joint = jnt2;
addBody(robot, body2, 'link1');

body3 = rigidBody('link3');
jnt3 = rigidBodyJoint('joint3', 'prismatic');
setFixedTransform(jnt3, [0, 0, 0, 0], 'dh');
body3.Joint = jnt3;
addBody(robot, body3, 'link2');

body4 = rigidBody('link4');
jnt4 = rigidBodyJoint('joint4', 'revolute');
setFixedTransform(jnt4, [0, 0, params.d4, 0], 'dh');
body4.Joint = jnt4;
addBody(robot, body4, 'link3');

T_matlab = getTransform(robot, q, 'link4', 'base');

%% 4. Kiem tra sai so dinh luong
max_error = max(abs(T_custom - T_matlab), [], 'all');
fprintf('Toa do khau cuoi tu tinh [x, y, z]: [%.4f, %.4f, %.4f] m\n', T_custom(1:3, 4));
fprintf('Sai so lon nhat so voi MATLAB Toolbox: %.2e\n', max_error);

%% 5. Ve mo hinh 3D
figure('Name', '4-DOF SCARA Forward Kinematics', 'Color', 'w');
plot3(pts(1,:), pts(2,:), pts(3,:), '-o', 'LineWidth', 3, 'MarkerSize', 8, ...
      'MarkerFaceColor', '#0072BD', 'Color', '#0072BD');
hold on; grid on; axis equal;
plot3(0, 0, 0, 'ks', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
plot3(pts(1,end), pts(2,end), pts(3,end), 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r');

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Mo hinh 3D tay may SCARA 4-DOF (Dong hoc thuan)');
legend('Cac khau robot', 'Goc toa do {0}', 'Diem tac dong cuoi {4}', 'Location', 'best');
xlim([-0.7 0.7]); ylim([-0.7 0.7]); zlim([0 0.5]);
view(135, 25);