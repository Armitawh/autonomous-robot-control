clc;
clear all;
restoredefaultpath;
addpath(genpath(pwd));

% Create Robot
R1 = RobotClass('json_fname', 'robot_0002.json');
% Uncomment below for using the real robot (it's simulation otherwise)
% R1.connect('192.168.1.1');

% Create World
W = WorldClass('fname', 'world_0002.json');

% Total duration and sampling time parameters
TotalTime = 2;
t_sampling = 0.02;

t_start = tic;
t_loop = tic;

%% Initialise data

% Initialise motor angular velocity controllers
control_right = MotorControl();
control_left = MotorControl();

% PID gains for right and left wheels
Kp = 0.014;  % Proportional gain
Ki = 0.22; % Integral gain
Kd = 0; % Derivative gain


Ti = (1/Ki);



Ts = t_sampling; % Sampling time

% Controller setpoints for right and left wheels
wR_set = 4;
wL_set = 9;

% Wheel angular velocities from the encoders
wR = 0;
wL = 0;

% Variables for storing PID errors and previous values
eR_prev = 0; % Previous error for right wheel
eL_prev = 0; % Previous error for left wheel
uR_prev = 0; % Previous control output for right wheel
uL_prev = 0; % Previous control output for left wheel

% Vectors for saving input/output data
wR_set_all = [];
wL_set_all = [];
wR_all = [];
wL_all = [];
time_all = [];

while toc(t_start) < TotalTime
    
    dt = toc(t_loop);
    
    if(dt >= t_sampling)              % execute code when desired sampling time is reached
        t_loop = tic;
        
        %% Velocity Form PID Controller for Right Wheel %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        eR = wR_set - wR; % Error for right wheel
        
        % Calculate the control output using the velocity form of PID
        uR = uR_prev + Kp * (eR * (1 + Ts / Ti) + Kd * (eR - eR_prev) / Ts);
        uR = max(min(uR,1),-1);
        % Update previous error and control output
        eR_prev = eR;
        uR_prev = uR;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %% Velocity Form PID Controller for Left Wheel %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        eL = wL_set - wL; % Error for left wheel
        
        % Calculate the control output using the velocity form of PID
        uL = uL_prev + Kp * (eL * (1 + Ts / Ti) + Kd * (eL - eL_prev) / Ts);
        uL = max(min(uL,1),-1);
        % Update previous error and control output
        eL_prev = eL;
        uL_prev = uL;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % Save data for plotting
        wR_set_all = [wR_set_all wR_set];
        wL_set_all = [wL_set_all wL_set];
        wR_all = [wR_all wR];
        wL_all = [wL_all wL];
        time_all = [time_all toc(t_start)];
        
        %% Do not edit this section
        % Update robot in this order: actuators, pose (in simulation), sensors
        actuator_signals = {'right motor', uR, 'left motor', uL};
        sensor_readings = R1.update(dt, W, 'kinematics', 'voltage_pwm', actuator_signals{:});
        
        % Update encoder velocity readings
        wR = sensor_readings('right encoder');
        wL = sensor_readings('left encoder');        
    end

    pause(0.001)
end

% Plot wheel angular velocities with time on the x-axis
figure;
plot(time_all, wR_all, 'r', 'LineWidth', 2); % Right wheel actual velocity
hold on;
plot(time_all, wL_all, 'b', 'LineWidth', 2); % Left wheel actual velocity
plot(time_all, wR_set_all, 'r--', 'LineWidth', 2); % Right wheel setpoint
plot(time_all, wL_set_all, 'b--', 'LineWidth', 2); % Left wheel setpoint

% Labeling the axes and adding the title
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Wheel Angular Velocities vs Time');

% Add legends to the plot
legend('Right Wheel Actual', 'Left Wheel Actual', 'Right Wheel Setpoint', 'Left Wheel Setpoint');
grid on;
