function [ xi, Omg] = eif_localization( xi, Omg, u, Z, f, h, F, G, H, landmarks, dt, R, Q)
%EIF_LOCALIZATION EIF Localization with Known Correspondences
%   EIF Localization of 3DOF planar robot using a velocity model.
%   See Table 7.2 (p. 204) & Table 3.6 (p. 76)
%   Inputs:
%   xi - transformed mean vector inv(Cov)*mu
%   Omg - inverse covariance matrix
%   u - control (in this case (v,w))
%   Z - measurements for all landmarks seen (None is ok!) i.e. []
%   f - f(x, u, dt) propogation function handle
%   h - h(x, u, m, dt) measurement function handle (note: m is the
%   landmark's position)
%   F - F(x, u, dt) df/dx state propogation jacobian function handle
%   G - G(x, u, dt) df/du input propogation jacobian function handle
%   H - H(x, u, m, dt) dg/dx measurement jacobian function handle
%   landmarks - positions of landmarks seen (None is ok!) i.e. []
%   dt - timestep
%   alphas - params on contol noise
%   params - contain parameters of measurement noise
%   
% -------------------------------------------------------------------------
% Noise and system setup
% -------------------------------------------------------------------------
% Breakout variables for convenience
v = u(1);
w = u(2);

% linearized state transition function (A) -- eq (7.8)
mu = Omg\xi;
Ft = F(mu, u, dt);

% linearized input matrix (B) -- eq (7.11)
Gt = G(mu, u, dt);

% Inverse of measurement noise for efficiency
Qinv = inv(Q);
% =========================================================================

% -------------------------------------------------------------------------
% Prediction Step
% -------------------------------------------------------------------------
Omgbar = inv(Ft*inv(Omg)*Ft.' + R);
mubar = f(mu,u,dt);
xibar = Omgbar*mubar;
% =========================================================================

% -------------------------------------------------------------------------
% Measurement updates
% -------------------------------------------------------------------------
N = size(Z,2);
Zhat = zeros(2,N);
% Note: if N = 0, there will be no measurement update
for i = 1:N
    % Position of the true landmark from the map
    m = landmarks(:,i);

    % Given the 'true' landmark and our current perceived position,
    % what is the measurement we should have gotten from the sensor?
    zhat = h(mubar, u, m, dt);

    % Build the linearized measurement model (C) -- eq (7.14)
    Ht = H(mubar, u, m, dt);
    
    % Innovation covariance
    S = Ht/Omgbar*Ht' + Q;
    
    % Gate the residual
    r = Z(:,i) - zhat;
    if r'/S*r > 1.5^2, continue; end
    
    % Heading wrapping
    if r(2) > pi
        r(2) = r(2) - 2*pi;
    elseif r(2) < -pi
        r(2) = r(2) + 2*pi;
    end
    
    % Kalman update
    Omgbar = Omgbar + Ht.'*Qinv*Ht;
    xibar = xibar + Ht.'*Qinv*(r + Ht*mubar);

    % Save for later
    Zhat(:,i) = zhat;
end
% =========================================================================

% Update belief
xi = xibar;
Omg = Omgbar;
end