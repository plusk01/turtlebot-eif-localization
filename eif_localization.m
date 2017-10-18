function [ xi, Omg] = eif_localization( xi, Omg, u, Z, f, g, F, G, H, landmarks, dt, alphas, params)
%EIF_LOCALIZATION EIF Localiation with Known Correspondences
%   EIF Localization of 3DOF planar robot using a velocity model.
%   See Table 7.2 (p. 204) & Table 3.6 (p. 76)
%   Inputs:
%   xi - transformed mean vector inv(Cov)*mu
%   Omg - inverse covariance matrix
%   u - control (in this case (v,w))
%   Z - measurements for all landmarks seen (None is ok!) i.e. []
%   f - f(x, u, dt) propogation function handle
%   g - g(x, u, m, dt) measurement function handle (note: m is the
%   landmark's position
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

% control noise cov matrix (to be mapped to state space w/ V) -- eq (7.10)
M = diag([alphas(1)*v^2 + alphas(2)*w^2 alphas(3)*v^2 + alphas(4)*w^2]);

% measurement noise (R) -- eq (7.15)
Q = diag([params.sigma_r^2 params.sigma_phi^2 params.sigma_s^2]);
Qinv = inv(Q);
% =========================================================================

% -------------------------------------------------------------------------
% Prediction Step
% -------------------------------------------------------------------------
Omgbar = inv(Ft*inv(Omg)*Ft.' + Gt*M*Gt.');
mubar = f(mu,u,dt);
xibar = Omgbar*mubar;
% =========================================================================

% -------------------------------------------------------------------------
% Measurement updates
% -------------------------------------------------------------------------
N = size(Z,2);
Zhat = zeros(3,N);
% Note: if N = 0, there will be no measurement update
for i = 1:N
    % Known correspondence variable to choose the 'truth' landmark
    m = landmarks(:,i);

    % Given the 'true' landmark and our current perceived position,
    % what is the measurement we should have gotten from the sensor?
    zhat = g(mubar, u, m, dt);

    % Build the linearized measurement model (C) -- eq (7.14)
    Ht = H(mubar, u, m, dt);

    % Kalman update
    Omgbar = Omgbar + Ht*Qinv*Ht;
    xibar = xibar + Ht*Qinv*(Z(:,i) - zhat + Ht*mubar);

    % Save for later
    Zhat(:,i) = zhat;
end
% =========================================================================

% Update belief
xi = xibar;
Omg = Omgbar;
end