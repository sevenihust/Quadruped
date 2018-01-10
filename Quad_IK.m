% calculate inverse kinematics of Quadrapeds robot
function [base_position, base_rotation, joints_angles, err_val, exitflag] = Quad_IK(des_base_position, des_base_rotation, des_abs_foot_pos, base_movable)
% @param des_base_position: target position of mobile base w.r.t. global coordinate frame, 3x1 vector
% @param des_base_rotation: target orientation of mobile base w.r.t. global coordinate frame, 3x3 matrix
% @param des_abs_foot_pos: target feet positions w.r.t. global coordinate frame, 3x4 matrix
% @param base_movable: boolean, if ture, 6 DoF of base will no consider
% into the optimal problem; if false, consider the whole robot motion as an
% optimal problem

% @return base_position: real position of mobile base 
% @return base_rotation: real orientation of mobile base
% @return joints_angles: joints angles that can realize target pose of
% mobile base and feet as much as possible; at the same time, achive
% minimal diviations
% @return err_val: square errors between desired pose and real pose
% @return exitflag: exiting condition of optimal problem, refer to matlab
% document here: https://www.mathworks.com/help/optim/ug/fmincon.html#outputarg_exitflag

quad_param;

global desired_abs_pos;

joints_angles = zeros(num_legs, max(nums_joints));

% des_base_position = [0; 0; 2];
% des_base_rotation = eye(3);

% base_movable = true;

if ~base_movable
    err_vals = zeros(1, num_legs);
    exitflags = zeros(1, num_legs);
    base_position = des_base_position;
    base_rotation = des_base_rotation;
    for i = 1:num_legs
        rel_pos_des = des_base_rotation'*(des_abs_foot_pos(:, i) - des_base_position);
        [joints_angles(i, :), err_vals(i), exitflags(i)] = Quad_LegIK(rel_pos_des, zeros(3, 1), i);
    end
%     joints_angles
%     err_vals
    err_val = sum(err_vals)
    exitflag = min(exitflags)
else
    desired_abs_pos = des_abs_foot_pos;
    myoptions = optimset('Algorithm','active-set', 'TolFun',1e-12,...
        'TolCon',1e-12, 'TolX',1e-12, 'TolConSQP',1e-12,...
        'MaxFunEvals',1e+10, 'MaxIter',1e+10, 'MaxSQPIter',1e+10);
    total_num_joints = sum(nums_joints);
    x0 = zeros(total_num_joints+6, 1);
    lbs = [zeros(6, 1)-100; joints_angle_bounds(1, :, 1)'; ...
        joints_angle_bounds(1, :, 2)'; joints_angle_bounds(1, :, 3)';...
        joints_angle_bounds(1, :, 4)'];
    ubs = [zeros(6, 1)+100; joints_angle_bounds(2, :, 1)'; ...
        joints_angle_bounds(2, :, 2)'; joints_angle_bounds(2, :, 3)';...
        joints_angle_bounds(2, :, 4)'];
    
    [x, err_val, exitflag] = fmincon(@obj, x0, [], [], [], [], lbs, ubs, [], myoptions);
    err_val
    exitflag
    base_position = x(1:3)
    base_rotation = exp2R(x(4:6))
    joints_angles(1, 1:3) = x(7:9)';
    joints_angles(2, 1:3) = x(9:11)';
    joints_angles(3, 1:3) = x(12:14)';
    joints_angles(4, 1:3) = x(14:16)';
    joints_angles
end

%% objective function for IK
function val = obj(x)
global desired_abs_pos;
quad_param;
base_position = x(1:3);
base_rotation = exp2R(x(4:6));
count = 6;
val = 0;
for i = num_legs
    abs_pos = Quad_LegFK(base_position, base_rotation, x(count+1:count+nums_joints(i)), i);
    err_pos = abs_pos - desired_abs_pos(:, i);
    val = val + err_pos'*err_pos;
    count = count + nums_joints(i);
end

%% exponential coordinates to SO(3)
function R = exp2R(dir)
theta = norm(dir);
if theta > 1e-13
    omega = dir/theta;
    omega_hat = [0 -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0];
    R = eye(3) + omega_hat*sin(theta) + omega_hat*omega_hat*(1-cos(theta));
else
    R = eye(3);
end

%% SO(3) to exponential coordinates
function dir = R2exp(R)
theta = acos((trace(R)-1)*0.5);
if theta > 1e-13
    dir = [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)]*(theta*0.5/sin(theta));
else
    dir = zeros(3,1);
end