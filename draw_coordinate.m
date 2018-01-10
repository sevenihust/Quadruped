function [] = draw_coordinate(p, R)

% @para p position w.r.t. global frame
% @para R rotation matrix w.r.t. global fram
scale = 0.02;
plot3([p(1); p(1) + scale*R(1, 1)],[p(2); p(2) + scale*R(2, 1)],[p(3); p(3) + scale*R(3, 1)],'Color','r','LineWidth',1)
plot3([p(1); p(1) + scale*R(1, 2)],[p(2); p(2) + scale*R(2, 2)],[p(3); p(3) + scale*R(3, 2)],'Color','g','LineWidth',1)
plot3([p(1); p(1) + scale*R(1, 3)],[p(2); p(2) + scale*R(2, 3)],[p(3); p(3) + scale*R(3, 3)],'Color','b','LineWidth',2)