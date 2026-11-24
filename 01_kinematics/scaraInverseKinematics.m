function q = scaraInverseKinematics(target_pose, params, elbow_dir)
% scaraInverseKinematics Giai dong hoc nghich tay may SCARA 4-DOF
% Dau vao:
%   target_pose - Vector [x, y, z, phi] (m, m, m, rad)
%   params      - Struct kich thuoc hinh hoc L1, L2, d1, d4
%   elbow_dir   - Chon cau hinh khuyu tay: +1 (theta2 >= 0) hoac -1 (theta2 < 0)
% Dau ra:
%   q           - Vector bien khop [theta1, theta2, d3, theta4]

x   = target_pose(1);
y   = target_pose(2);
z   = target_pose(3);
phi = target_pose(4);

L1 = params.L1;
L2 = params.L2;

% 1. Kiem tra gioi han vung lam viec tren mat phang XY
cos_theta2 = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2);
if abs(cos_theta2) > 1 + 1e-9
    error('Muc tieu [%.3f, %.3f] nam ngoai tam voi cua tay may SCARA!', x, y);
end
cos_theta2 = max(min(cos_theta2, 1), -1); % Chong sai so lam tron so hoc

% 2. Tinh goc khuyu tay theta2 theo cau hinh duoc chon (+1 hoac -1)
theta2 = elbow_dir * acos(cos_theta2);

% 3. Tinh goc vai theta1 bang phuong phap hinh hoc
k1 = L1 + L2 * cos(theta2);
k2 = L2 * sin(theta2);
theta1 = atan2(y, x) - atan2(k2, k1);

% 4. Tinh hanh trinh truc Z (d3) va goc xoay dau kep (theta4)
d3 = params.d1 - params.d4 - z;
theta4 = theta1 + theta2 - phi;

% Chuan hoa cac goc quay ve khoang [-pi, pi]
wrap = @(ang) atan2(sin(ang), cos(ang));
q = [wrap(theta1), wrap(theta2), d3, wrap(theta4)];
end