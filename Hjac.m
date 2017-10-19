function H = h(x, u, m, dt)
%h Linearized measurement model
%   Thrun, eq 7.14 (p. 207)

% For convenience
q = norm(m - x(1:2))^2;

% Build the linearized measurement model (C) -- eq (7.14)
H = [-(m(1)-x(1))/sqrt(q) -(m(2)-x(2))/sqrt(q)  0;...
      (m(2)-x(2))/q       -(m(1)-x(1))/q       -1];
end
