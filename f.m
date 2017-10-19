function mubar = f(mu, u, dt)
% inputs
v = u(1);
w = u(2);

% states
x = mu(1);
y = mu(2);
theta = mu(3);

% propagate
x_new = x + (v * cos(theta)) * dt;
y_new = y + (v * sin(theta)) * dt;
theta_new = theta + w * dt;

% return mubar
mubar = [x_new, y_new, theta_new]';
end

% assuming simplified system dynamics:
% x' = x + (v * cos(theta)) * dt
% y' = y + (v * sin(theta)) * dt
% theta' = theta + w * dt