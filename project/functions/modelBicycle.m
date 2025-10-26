function q_new = modelBicycle(q, u, T)
% MODELBICYCLE Simulates bicycle model kinematics over time T.
%
%   Q_NEW = MODELBICYCLE(Q, U, T) updates the state of a vehicle using the
%   kinematic bicycle model given the current state Q, control input U, and
%   time duration T.
%
%   Inputs:
%       Q - Current state vector [x, y, theta], where:
%           x, y   - Position of the vehicle in 2D space
%           theta  - Heading angle (radians)
%
%       U - Control input vector [speed, steering_angle], where:
%           speed           - Forward velocity
%           steering_angle  - Steering angle (radians)
%
%       T - Duration over which to apply the control input (seconds)
%
%   Output:
%       Q_NEW - New state vector [x, y, theta] after time T.

% States
x = q(1);
y = q(2);
theta = q(3);

% Inputs
speed = u(1);
steering_angle = u(2);

% Parameters
wheelbase = 2.0;

time = linspace(0, T, 100);
dt = diff(time(1:2));
for t = time
    x = x + speed * cos(theta) * dt;
    y = y + speed * sin(theta) * dt;
    theta = theta + (speed / wheelbase) * tan(steering_angle) * dt;
end

q_new = [x y theta];

end