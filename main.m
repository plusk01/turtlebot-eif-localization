%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% EIF Localization
%
% Chapter 3.5, Table 3.6 (p. 76)
% Probabilistic Robotics, Thrun et al.
%
% Assumptions:
%   - Known correspondences
%   - Velocity motion model (Unicycle Model)
%   - Range/bearing measurement model (Ch. 6.6)
%
% Parker Lusk, Devon Morris, Jesse Wynn, Vallan Sherrod BYU
% 18 Oct 2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear, clc;
% === Robot Parameters ====================================================

load('processed_data.mat')

% Range/bearing sensor parameters
sensor.sigma_r = 0.1;      % m     range std dev
sensor.sigma_phi = 0.05;   % rad   bearing std dev

% Velocity motion model noise parameters
alphas = [...
            0.1 0.01...     % translational error
            0.01 0.1...     % angular error
            0 0;...         % final orientation error
         ];

% Initial state
x0 = pos_odom_se2(:,1);

% =========================================================================

% === World Parameters ====================================================

% Window of the field
field.x = [-2, 8];   % m
field.y = [-2, 16];   % m


% =========================================================================

% Draw the environment
figure(1), clf; hold on;
scatter(landmarks(:,1), landmarks(:,2), 'k*');
axis([field.x(1) field.x(2) field.y(1) field.y(2)]);
plot(pos_odom_se2(1,:),pos_odom_se2(2,:))
axis square; grid on

% Initializes the robot starting point (x, y, theta (deg))
bot = drawRobot(x0, []);

% Setup EIF initial conditions
x = [0 0 0]';
Omg = eye(3);

xi = Omg*x;

% Save variables for later
Xhat = zeros(3, length(odom_t));
OO   = zeros(3,3,length(odom_t));

%index that will keep track of measurements
l_t_idx = 1;

for i = 1:length(odom_t)-1
    
    %determine timestep
    Ts = odom_t(i+1) - odom_t(i);
    
    Z = [];
    landmarks_in = [];
    
    %determine if their is a measurment update and grab the data if
    %necessary
    if l_t_idx <= length(l_time) %this keeps it from seg faulting when it has grabbed the last measurement data
        if l_time(l_t_idx) < odom_t(i+1)
            bearings = l_bearing(:,l_t_idx);
            ranges = l_depth(:,l_t_idx);       
            for j = 1:length(bearings)
                if ~isnan(bearings(j))
                    Z = [Z [ranges(j) bearings(j)]'];
                    landmarks_in = [landmarks_in landmarks(j,:)'];
                end
            end
            l_t_idx = l_t_idx + 1;
        end   
    end
    
    % Localize robot using an EKF and landmark measurements
    [xi, Omg] = eif_localization(xi, Omg, vel_odom(:,i), Z, @f, @g, @F, @G, @H, landmarks_in, Ts, alphas, sensor);
   
    x = Omg\xi;
    
    %plot robot where we estimated its location
    bot = drawRobot(x, bot);
    
    % For plotting
    Xhat(:,i) = x;
    OO(:,:,i) = Omg;

end

figure(2), clf;
subplot(311); plot(t, Xhat(1,:)); ylabel('x [m]');
title('Estimate'); legend('Estimate','Location','northwest');
subplot(312); plot(t, Xhat(2,:)); ylabel('y [m]')
subplot(313); plot(t, Xhat(3,:)); ylabel('\psi [rad]');
xlabel('Time [s]')

figure(1),
plot(Xhat(1,:),Xhat(2,:));


% -------------------------------------------------------------------------