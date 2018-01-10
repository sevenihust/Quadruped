% Quadrapeds robot, leg inverse kinematics
% forming a optimal problem to solve joints that realize target pose as
% much as possible

function [joint_angles, err_val, exitflag] = Quad_LegIK(rel_pos_des, thetas0, leg_no)

% @param rel_pos_des: desired foot position relative to base coordinate, 3x1 vector
% @param thetas0: initial joint angles to start optimization, 3x1 vector
% @param leg_no: leg identifier [1, 2, 3, 4] --> {'fr', 'fl', 'rr', 'rl'}, int

% @return joint_angles: 3x1 vector, joint angles can realize leg pose
% @return err_value: double, error between p_goal and real position of foot FK(joint_angles)
% @return exitflag: init, indicates terminate condition of optimization

quad_param;

global joints_p0i;
global joints_R0i;
global desired_rel_pos;
global num_joints;
global no_leg;

no_leg = leg_no;
joints_p0i = p0(:, :, leg_no);
joints_R0i = R0;
desired_rel_pos = rel_pos_des;
num_joints = nums_joints(leg_no);
myoptions = optimset('Algorithm','active-set', 'TolFun',1e-13,...
    'TolCon',1e-13, 'TolX',1e-13, 'TolConSQP',1e-13, 'MaxFunEvals',1e+10,...
    'MaxIter',1e+10, 'MaxSQPIter',1e+10, 'Display','off');
if length(thetas0) ~= num_joints
    thetas0 = zeros(num_joints, 1);
end
lbs = joints_angle_bounds(1, :, leg_no);
ubs = joints_angle_bounds(2, :, leg_no);
[thetas, err_val, exitflag] = fmincon(@obj, thetas0, [], [], [], [], lbs, ubs, [], myoptions);
joint_angles = thetas';
% err_val
% exitflag

%% objective function for IK
function val = obj(thetas)
global joints_p0i;
global joints_R0i;
global desired_rel_pos;
global num_joints;
global no_leg;

pi = zeros(3, 1);
Ri = eye(3);
for i = 1:num_joints
    pi = Ri*joints_p0i(:, i) + pi;
    c = cos(thetas(i));
    s = sin(thetas(i));
    Ri = Ri*joints_R0i(:, :, i, no_leg) * [c -s 0; s c 0; 0 0 1];
end
rel_pos = Ri*joints_p0i(:, num_joints + 1) + pi;
err_pos = rel_pos - desired_rel_pos;
val = err_pos' * err_pos;