function Ft = F(mu, u, dt)

% inputs
v = u(1);
w = u(2);

% states
x = mu(1);
y = mu(2);
theta = mu(3);

% state propogation jacobian (df/dx)
Ft = [1, 0, -v * sin(theta) * dt;
      0, 1, v * cos(theta) * dt;
      0, 0, 1];
end

% Jacobians assuming the simplified system dynamics:
% x' = x + (v * cos(theta)) * dt
% y' = y + (v * sin(theta)) * dt
% theta' = theta + w * dt