function z = h(x, u, m, dt)
%H Non-linear measurement model
%   Thrun, eq 6.40 (p. 178)

% Generate possible measurement given state, eq (7.12)
z = [...
      norm(m - x(1:2));...
      atan2( m(2)-x(2) , m(1)-x(1) ) - x(3);...
    ];
end

