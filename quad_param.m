% angle offset between frame of joint1 and frame of mobile base
theta_offset = [pi/4; 3*pi/4; -pi/4; -3*pi/4]; % in the order of fr -> fl -> rr -> rl
l0 = 0.152/2; % lenght of limb0 (between joint 1 and mobile base)
l1 = 0.028; % lenght of limb1 (between joint 1 and 2)
l2 = 0.057; % lenght of limb2 (between joint 2 and 3)
l3 = 0.088 + 0.0165 + 0.004; % lenght of limb3 (between joint 3 and foot tip)

num_legs = 4;
nums_joints = [3, 3, 3, 3];
% syms a0 a1 a2 a3;
% syms l0 l1 l2 l3 l4;

for i = 1:4
    cos0(i) = cos(theta_offset(i));
    sin0(i) = sin(theta_offset(i));
end
%% relative orietation between joints in the case of zero joint angles
%%%legs' profiles are identical
R0(:, :, 1, 1) = [cos(pi/4), -sin(pi/4), 0; sin(pi/4), cos(pi/4), 0; 0, 0, 1]; % first joint w.r.t. float base, with an offset angle of pi/4
R0(:, :, 2, 1) = [0 1 0; 0 0 -1; -1 0 0]; % second joint w.r.t. first joint, rotate about x
R0(:, :, 3, 1) = eye(3); % third joint w.r.t. second joint, rotate about z
R0(:, :, 4, 1) = eye(3); % foot w.r.t. third joint, rotate about y

R0(:, :, 1, 2) = [cos(-pi/4), -sin(-pi/4), 0; sin(-pi/4), cos(-pi/4), 0; 0, 0, 1]; % first joint w.r.t. float base, with an offset angle of pi/4
R0(:, :, 2, 2) = [0 -1 0; 0 0 -1; -1 0 0]; % second joint w.r.t. first joint, rotate about x
R0(:, :, 3, 2) = eye(3); % third joint w.r.t. second joint, rotate about z
R0(:, :, 4, 2) = eye(3); % foot w.r.t. third joint, rotate about y

R0(:, :, 1, 3) = [cos(-pi/4), -sin(-pi/4), 0; sin(-pi/4), cos(-pi/4), 0; 0, 0, 1]; % first joint w.r.t. float base, with an offset angle of pi/4
R0(:, :, 2, 3) = [0 1 0; 0 0 -1; -1 0 0]; % second joint w.r.t. first joint, rotate about x
R0(:, :, 3, 3) = eye(3); % third joint w.r.t. second joint, rotate about z
R0(:, :, 4, 3) = eye(3); % foot w.r.t. third joint, rotate about y

R0(:, :, 1, 4) = [cos(pi/4), -sin(pi/4), 0; sin(pi/4), cos(pi/4), 0; 0, 0, 1]; % first joint w.r.t. float base, with an offset angle of pi/4
R0(:, :, 2, 4) = [0 -1 0; 0 0 -1; -1 0 0]; % second joint w.r.t. first joint, rotate about x
R0(:, :, 3, 4) = eye(3); % third joint w.r.t. second joint, rotate about z
R0(:, :, 4, 4) = eye(3); % foot w.r.t. third joint, rotate about y

%% relative position of one joint in the coordinate frame of its previous joint
%%% front right leg
p0(:, 1, 1) = [l0*cos0(1); l0*sin0(1); 0];
p0(:, 2, 1) = [l1; 0; 0];
p0(:, 3, 1) = [0; l2; 0];
p0(:, 4, 1) = [l3; 0; 0];
%%% front left leg
p0(:, 1, 2) = [l0*cos0(2); l0*sin0(2); 0];
p0(:, 2, 2) = [-l1; 0; 0];
p0(:, 3, 2) = [0; l2; 0];
p0(:, 4, 2) = [l3; 0; 0];
%%% rare right leg
p0(:, 1, 3) = [l0*cos0(3); l0*sin0(3); 0];
p0(:, 2, 3) = [l1; 0; 0];
p0(:, 3, 3) = [0; l2; 0];
p0(:, 4, 3) = [l3; 0; 0];
%%% rare left leg
p0(:, 1, 4) = [l0*cos0(4); l0*sin0(4); 0];
p0(:, 2, 4) = [-l1; 0; 0];
p0(:, 3, 4) = [0; l2; 0];
p0(:, 4, 4) = [l3; 0; 0];
%% joint angle ranges
joints_angle_bounds(:, :, 1) = [-pi/2, -pi/2, -pi/2; pi/2, pi/2, pi/2];
joints_angle_bounds(:, :, 2) = [-pi/2, -pi/2, -pi/2; pi/2, pi/2, pi/2];
joints_angle_bounds(:, :, 3) = [-pi/2, -pi/2, -pi/2; pi/2, pi/2, pi/2];
joints_angle_bounds(:, :, 4) = [-pi/2, -pi/2, -pi/2; pi/2, pi/2, pi/2];
%% draw the robot frame
% figure
% % axis([0 10 0 10 -5 5]) %set x y z axis size according to the problem
% axis equal
% hold on
% view(3)
% grid
% %daspect([1,1,1])


% %camzoom(1)
% camproj orthographic
% rotate3d on
% plot3([l0*cos0(1), l0*cos0(3)], [l0*sin0(1), l0*sin0(3)], [0, 0], 'Color', 'c', 'LineWidth', 2.0);
% plot3([l0*cos0(4), l0*cos0(3)], [l0*sin0(4), l0*sin0(3)], [0, 0], 'Color', 'c', 'LineWidth', 2.0);
% plot3([l0*cos0(4), l0*cos0(2)], [l0*sin0(4), l0*sin0(2)], [0, 0], 'Color', 'c', 'LineWidth', 2.0);
% plot3([l0*cos0(1), l0*cos0(2)], [l0*sin0(1), l0*sin0(2)], [0, 0], 'LineStyle', '--', 'Color', 'g', 'LineWidth', 2.0);
% plot3([0;0.03],[0;0],[0;0],'r','LineWidth',3)
% plot3([0;0],[0;0.03],[0;0],'g','LineWidth',3)
% plot3([0;0],[0;0],[0;0.03],'b','LineWidth',3)