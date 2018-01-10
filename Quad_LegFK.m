% Quadrapeds robot, leg forward kinematics

function [abs_position, rel_position, abs_rotation, rel_rotation] = Quad_LegFK(base_position, base_rotation, joint_angles, leg_no)
% @param base_position: position of mobile base w.r.t. world/global coordinate frame, 3x1 vector
% @param base_rotation: orientation of mobile base w.r.t. world.global coordinate frame, 3x3 matrix
% @param joint_angles: 3x1 vector
% @param leg_no: leg identifier [1, 2, 3, 4] --> {'fr', 'fl', 'rr', 'rl'}, int

% @return abs_position: foot position w.r.t. world/global coordinate frame, 3x1 vector
% @return rel_position: foot position w.r.t. base coordinate frame, 3x1 vector
% @return abs_rotation: foot orientation w.r.t. world/global coordinate frame, 3x3 matrix
% @return rel_rotation: foot orientation w.r.t. base coordinate frame, 3x3 matrix

quad_param;

num_joints = nums_joints(leg_no);

rel_position = p0(:, num_joints + 1, leg_no);
rel_rotation = R0(:, :, num_joints + 1, leg_no);

for i = num_joints:-1:1
    c_theta = cos(joint_angles(i));
    s_theta = sin(joint_angles(i));
    Ri = R0(:, :, i, leg_no)*[c_theta -s_theta 0; s_theta c_theta 0; 0 0 1];
    rel_position = Ri*rel_position + p0(:, i, leg_no);
    rel_rotation = Ri*rel_rotation;
end
abs_position = base_rotation*rel_position + base_position;
abs_rotation = base_rotation*rel_rotation;