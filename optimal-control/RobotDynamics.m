function xdot = RobotDynamics(xt,ut,h)
% Compute the next state using the non-linear Robot Dynamics
global Ih mh rh Ib mb rb g

%%% In the x-z plane
% Control input to torque
r1 = ut(1);
t1prev = xt(5);
t1dot = 2*r1 - 2*t1prev;
t1new = t1prev + h*t1dot;

% Previous state
q1 = xt(1:2);
q1dot = xt(3:4);

M1 = [mb+Ib/rb^2+mh -mh*rh*cos(q1(2))
      -mh*rh*cos(q1(2)) mh*rh^2+Ih];
C1 = [0 mh*rh*q1dot(2)*sin(q1(2))
      0 0];
G1 = [0
      -mh*rh*g*sin(q1(2))];
R1 = [1 rb]';

q1ddot = M1\(R1*t1new - G1 - C1*q1dot);

%%% In the y-z plane
% Control input to torque
r2 = ut(2);
t2prev = xt(10);
t2dot = 2*r2 - 2*t2prev;
t2new = t2prev + h*t2dot;

% Previous state
q2 = xt(6:7);
q2dot = xt(8:9);

M2 = [mb+Ib/rb^2+mh -mh*rh*cos(q2(2))
      -mh*rh*cos(q2(2)) mh*rh^2+Ih];
C2 = [0 mh*rh*q2dot(2)*sin(q2(2))
      0 0];
G2 = [0
      -mh*rh*g*sin(q2(2))];
R2 = [1 rb]';

q2ddot = M2\(R2*t2new - G2 - C2*q2dot);

% Next State
xdot = [q1dot; q1ddot; t1dot; q2dot; q2ddot; t2dot];

end

