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

% Process Noise
R = diag([0.001^2   0.001^2   0.005^2]); % (x, y, theta)

% ArUco Measurement Noise
Q = diag([0.01^2   0.01^2]); % (range, bearing)

% Initial state
x0 = pos_odom_se2(:,1);

% =========================================================================

% === World Parameters ====================================================

% Window of the field
field.x = [-6, 12];   % m
field.y = [-2, 16];   % m

% =========================================================================

% Draw the environment
figure(1), clf; hold on;
scatter(landmarks(:,1), landmarks(:,2), 'k*');
axis([field.x(1) field.x(2) field.y(1) field.y(2)]);
plot(pos_odom_se2(1,:),pos_odom_se2(2,:))
axis square; grid on

% Initializes the robot starting point (x, y, theta (deg))
bot = drawRobot(1, x0, []);

% Setup EIF initial conditions
x = [0 0 0]';
x = x0;
Omg = eye(3);

xi = Omg*x;

% Save variables for later
Xhat = zeros(3, length(odom_t));
PP   = zeros(3,3,length(odom_t));

%index that will keep track of measurements
l_t_idx = 1;

for i = 1:length(odom_t)-1
    
    %determine timestep
    Ts = odom_t(i+1) - odom_t(i);
    
    Z = [];
    landmarks_in = [];
    
    %determine if there is a measurement update and grab the data if
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
        
    % Localize robot using an EIF and landmark measurements
    [xi, Omg] = eif_localization(xi, Omg, vel_odom(:,i), Z, @f, @h, ...
                    @Fjac, @Gjac, @Hjac, landmarks_in, Ts, R, Q);
   
    % 'unparameterize' to get state and error covariance
    xhat = Omg\xi;
    P = inv(Omg);
    
    % plot robot where we estimated its location
    bot = drawRobot(mod(i,20)==0,xhat, bot);
    
    % For plotting
    Xhat(:,i) = xhat;
    PP(:,:,i) = P;

end

% Because of the way we are handling the timesteps (i.e., non-causal) get
% rid of the last datapoint for the purposes of plotting
Xhat = Xhat(:,1:end-1);
PP = PP(:,1:end-1);
t = odom_t(1:end-1);

figure(2), clf;
subplot(311); plot(t, Xhat(1,:)); ylabel('x [m]');
title('Estimate'); legend('Estimate','Location','northwest');
subplot(312); plot(t, Xhat(2,:)); ylabel('y [m]')
subplot(313); plot(t, Xhat(3,:)); ylabel('\psi [rad]');
xlabel('Time [s]')

figure(1),
plot(Xhat(1,:),Xhat(2,:));


% -------------------------------------------------------------------------