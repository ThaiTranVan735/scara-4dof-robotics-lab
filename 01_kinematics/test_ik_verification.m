clear; clc; close all;

%% 1. Khai bao thong so robot (giu nguyen nhu bai Dong hoc thuan)
params.L1 = 0.35;
params.L2 = 0.25;
params.d1 = 0.30;
params.d4 = 0.08;

% Dat toa do muc tieu can gap: [x (m), y (m), z (m), phi (rad)]
target_pose = [0.40, 0.25, 0.10, pi/4];

%% 2. Giai Dong hoc nghich cho ca 2 cau hinh khuyu tay
q_elbow_down = scaraInverseKinematics(target_pose, params, +1); % Nghiem 1: theta2 >= 0
q_elbow_up   = scaraInverseKinematics(target_pose, params, -1); % Nghiem 2: theta2 < 0

%% 3. Nap nguoc 2 nghiem vao ham Dong hoc thuan de kiem chung do chinh xac
[T_down, pts_down] = scaraForwardKinematics(q_elbow_down, params);
[T_up,   pts_up]   = scaraForwardKinematics(q_elbow_up, params);

err_down = norm(T_down(1:3, 4) - target_pose(1:3)');
err_up   = norm(T_up(1:3, 4)   - target_pose(1:3)');

fprintf('Muc tieu yeu cau [x, y, z]: [%.4f, %.4f, %.4f] m\n', target_pose(1:3));
fprintf('Nghiem 1 (Elbow-Down) q = [%.2f, %.2f deg, %.3f m, %.2f deg] | Sai so: %.2e m\n', ...
    rad2deg(q_elbow_down(1)), rad2deg(q_elbow_down(2)), q_elbow_down(3), rad2deg(q_elbow_down(4)), err_down);
fprintf('Nghiem 2 (Elbow-Up)   q = [%.2f, %.2f deg, %.3f m, %.2f deg] | Sai so: %.2e m\n', ...
    rad2deg(q_elbow_up(1)), rad2deg(q_elbow_up(2)), q_elbow_up(3), rad2deg(q_elbow_up(4)), err_up);

%% 4. Ve 3D hai tu the cung cham 1 muc tieu
figure('Name', '4-DOF SCARA Inverse Kinematics - Dual Solutions', 'Color', 'w');
plot3(pts_down(1,:), pts_down(2,:), pts_down(3,:), '-o', 'LineWidth', 3, ...
      'Color', '#0072BD', 'MarkerFaceColor', '#0072BD');
hold on; grid on; axis equal;
plot3(pts_up(1,:), pts_up(2,:), pts_up(3,:), '--s', 'LineWidth', 2.5, ...
      'Color', '#D95319', 'MarkerFaceColor', '#D95319');
plot3(target_pose(1), target_pose(2), target_pose(3), 'rp', 'MarkerSize', 16, 'MarkerFaceColor', 'r');

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Hai cau hinh nghiem Dong hoc nghich (Elbow-Down vs. Elbow-Up)');
legend('Nghiem 1: Elbow-Down (\theta_2 > 0)', 'Nghiem 2: Elbow-Up (\theta_2 < 0)', ...
       'Diem muc tieu (Target)', 'Location', 'best');
xlim([-0.6 0.7]); ylim([-0.6 0.7]); zlim([0 0.45]);
view(135, 30);