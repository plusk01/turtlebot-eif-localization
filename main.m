%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% EKF Localization
%
% Chapter 7.4, Table 7.2 (p. 204)
% Probabilistic Robotics, Thrun et al.
%
% Assumptions:
%   - Known correspondences
%   - Velocity motion model (Ch. 5.3)
%   - Range/bearing measurement model (Ch. 6.6)
%
% Parker Lusk, BYU
% 25 Sept 2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear, clc;
% === Robot Parameters ====================================================

% Range/bearing sensor parameters
sensor.sigma_r = 0.1;      % m     range std dev
sensor.sigma_phi = 0.05;   % rad   bearing std dev
sensor.sigma_s = 5;        % signature noise std dev

% Velocity motion model noise parameters
alphas = [...
            0.1 0.01...     % translational error
            0.01 0.1...     % angular error
            0 0;...         % final orientation error
         ];

% Initial state
x0 = [-5 -3 pi/2]';

% Simulation Timing
Ts = 0.1;       % Sample rate
Tf = 20;        % Simulation duration
t = 0:Ts:Tf;    % Time vector

% Velocity commands
vc = 1 + 0.5*cos(2*pi*(0.2).*t);
wc = -0.2 + 2*cos(2*pi*(0.6).*t);
% wc = zeros(size(t));
% =========================================================================

% === World Parameters ====================================================

% Dimensions of the field
field.w = 20;   % m
field.h = 20;   % m

% Landmark locations
field.landmarks = [6 4 100; -7 8 200; 6 -4 300]';
% =========================================================================

% Draw the environment
figure(1), clf; hold on;
scatter(field.landmarks(1,:), field.landmarks(2,:), 'k*');
axis([-field.w field.w -field.h field.h]/2);
axis square; grid on

% Initializes the robot starting point (x, y, theta (deg))
bot = drawRobot(x0, []);

% Setup true state and inputs
xtr = x0;
utr = [vc; wc];

% Setup EKF initial conditions
x = [0 0 0]';
P = eye(3);

% Known correspondence index
c = [1, 2, 3];

% Save variables for later
Xtr  = zeros(3, length(t));
Xhat = zeros(3, length(t));
PP   = zeros(3,3,length(t));
KKs  = cell(1,length(t));

for i = 1:length(t)
    
    % Move the robot
    [xtr, unoisy] = sim_kinematics(xtr, utr(:,i), Ts, alphas);
    bot = drawRobot(xtr, bot);

    % Read the sensor
    Z = range_bearing_sensor(xtr, field.landmarks, sensor);
    
    % Localize robot using an EKF and landmark measurements
    [x, P, KK] = ekf_localization(x, P, unoisy, Z, c, field.landmarks, Ts, alphas, sensor);
   
    % For plotting
    Xtr(:,i) = xtr;
    Xhat(:,i) = x;
    PP(:,:,i) = P;
    KKs(1,i) = {KK};

end

figure(2), clf;
subplot(311); plot(t, Xtr(1,:), t, Xhat(1,:)); ylabel('x [m]');
title('Truth vs Estimate'); legend('Truth', 'Estimate','Location','northwest');
subplot(312); plot(t, Xtr(2,:), t, Xhat(2,:)); ylabel('y [m]')
subplot(313); plot(t, Xtr(3,:), t, Xhat(3,:)); ylabel('\psi [rad]');
xlabel('Time [s]')

% Estimation error
figure(3), clf;
Xerr = Xtr - Xhat;
bound = 2;
sigma_x = sqrt(reshape(PP(1,1,:),[],1));
sigma_y = sqrt(reshape(PP(2,2,:),[],1));
sigma_psi = sqrt(reshape(PP(3,3,:),[],1));
subplot(311);
plot(t, Xerr(1,:)); hold on; title('Estimation Error');
plot(t, bound*sigma_x, 'r', 'LineWidth', 2);
plot(t, -bound*sigma_x, 'r', 'LineWidth', 2);
ylabel('x [m]'); legend('Error', '2-\sigma bound (95%)');
subplot(312);
plot(t, Xerr(2,:)); hold on; ylabel('y [m]'); 
plot(t, bound*sigma_y, 'r', 'LineWidth', 2);
plot(t, -bound*sigma_y, 'r', 'LineWidth', 2);
subplot(313);
plot(t, Xerr(3,:)); hold on; ylabel('\psi [rad]'); 
plot(t, bound*sigma_psi, 'r', 'LineWidth', 2);
plot(t, -bound*sigma_psi, 'r', 'LineWidth', 2);
xlabel('Time [s]');

% Kalman Gain
figure(4), clf;
L = size(field.landmarks, 2);
for i = 1:L
    
    Ks = zeros(3,3,length(t));
    for j = 1:length(t)
        KK = KKs{j};
        Ks(:,:,j) = KK(:,:,i);
    end
    
    subplot(L,1,i);
    plot(t,reshape(Ks, 9, []));
    ylabel(['Landmark ' int2str(i)]);
    
    if i == 1
        title('Kalman Gains');
    elseif i == L
        xlabel('Time [s]');
    end
end

% -------------------------------------------------------------------------