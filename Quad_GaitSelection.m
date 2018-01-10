%%% Quad Gait Pattern Selection

function [standard_gait] = Quad_GaitSelection(linear_vel, angular_vel)

% @param liear_vel: 1x2 vector, walking speed of turnning center(CoG);
% @param angular_vel: double, rotation speed of turning center;
% 
% @return walking_gait: int, type of static walking gait
% 1 - X-crawl
% 2 - RX-crawl
% 3 - Y-crawl
% 4 - RY-crwal
% 5 - O-rotation
% 6 - RO-rotation
v_x = linear_vel(1);
v_y = linear_vel(2);
% radius of the turning curve
if abs(angular_vel) < 1e-10
    angular_vel = 1e-10;
end
r = sqrt(v_x^2 + v_y^2)/abs(angular_vel);

% position of the turning center, locates at the ground. w.r.t. global or
% local frame?
Q = [0 -1 0; 1 0 0; 0 0 1]*[v_x/angular_vel; v_y/angular_vel; -z_CoG];

% calculate gaits selection region
foot_basic_position = zeros(3, 4); % w.r.t basic coordinate?
for i = 1:4
    xi = foot_basic_position(1, i);
    yi = foot_basic_position(2, i);
    y = -(xi/yi) * x + (xi^2 + yi^2)/yi;
end