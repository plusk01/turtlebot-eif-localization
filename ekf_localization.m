function [ mu, Sig, KK ] = ekf_localization( mu, Sig, u, Z, c, landmarks, dt, alphas, params )
%EKF_LOCALIZATION EKF Localiation with Known Correspondences
%   EKF Localization of 3DOF planar robot using a velocity model.
%   See Table 7.2 (p. 204)

% -------------------------------------------------------------------------
% Noise and system setup
% -------------------------------------------------------------------------
% Breakout variables for convenience
v = u(1);
w = u(2);
theta = mu(3);

% linearized state transition function (A) -- eq (7.8)
G = [...
    1 0 -(v/w)*cos(theta) + (v/w)*cos(theta+w*dt);...
    0 1 -(v/w)*sin(theta) + (v/w)*sin(theta+w*dt);...
    0 0  1];

% linearized input matrix (B) -- eq (7.11)
V = [...
    % row 1
    (-sin(theta)+sin(theta+w*dt))/w ...
        (v/w^2)*(sin(theta)-sin(theta+w*dt))+(v/w)*cos(theta+w*dt)*dt;...
    % row 2
    (cos(theta)-cos(theta+w*dt))/w  ...
        (-v/w^2)*(cos(theta)-cos(theta+w*dt))+(v/w)*sin(theta+w*dt)*dt;...
    % row 3
    0 dt];

% control noise cov matrix (to be mapped to state space w/ V) -- eq (7.10)
M = diag([alphas(1)*v^2 + alphas(2)*w^2 alphas(3)*v^2 + alphas(4)*w^2]);

% measurement noise (R) -- eq (7.15)
Q = diag([params.sigma_r^2 params.sigma_phi^2 params.sigma_s^2]);
% =========================================================================

% -------------------------------------------------------------------------
% Prediction Step
% -------------------------------------------------------------------------
mubar = mu + [...
                -(v/w)*sin(theta) + (v/w)*sin(theta+w*dt);...
                 (v/w)*cos(theta) - (v/w)*cos(theta+w*dt);...
                 w*dt];
             
Sigbar = G*Sig*G.' + V*M*V.';
% =========================================================================

% -------------------------------------------------------------------------
% Measurement updates
% -------------------------------------------------------------------------
N = size(Z,2);
Zhat = zeros(3,N);
SS = zeros(3,3,N);
KK = zeros(3,3,N);
for i = 1:N
    % Known correspondence variable to choose the 'truth' landmark
    m = landmarks(:,c(i));
    
    % For convenience
    q = norm(m(1:2) - mubar(1:2))^2;
    
    % Given the 'true' landmark and our current perceived position,
    % what is the measurement we should have gotten from the sensor?
    zhat = [sqrt(q);...
            atan2(m(2)-mubar(2),m(1)-mubar(1)) - mubar(3);...
            m(3)];
        
    % Build the linearized measurement model (C) -- eq (7.14)
    H = [-(m(1)-mubar(1))/sqrt(q) -(m(2)-mubar(2))/sqrt(q)  0;...
          (m(2)-mubar(2))/q       -(m(1)-mubar(1))/q       -1;...
          0                        0                        0];

   % Kalman update
   S = H*Sigbar*H.' + Q;
   K = Sigbar*H.'/(S);
   mubar = mubar + K*(Z(:,i) - zhat);
   Sigbar = (eye(size(Sigbar)) - K*H)*Sigbar;
   
   % Save for later
   Zhat(:,i) = zhat;
   SS(:,:,i) = S;
   KK(:,:,i) = K;
end
% =========================================================================

% Update belief
mu = mubar;
Sig = Sigbar;
end

