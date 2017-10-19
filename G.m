function Gt = G(mu, u, dt)

% inputs
v = u(1);
w = u(2);

% heading
theta = mu(3);

% input propogation jacobian (df/du). I think these are right??? They seem a
% little too simple
Gt = [cos(theta) * dt, 0;
      sin(theta) * dt, 0;
      0, dt];
end

% Jacobians assuming the simplified system dynamics:
% x' = x + (v * cos(theta)) * dt
% y' = y + (v * sin(theta)) * dt
% theta' = theta + w * dt