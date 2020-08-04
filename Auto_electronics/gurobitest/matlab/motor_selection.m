% load data
data = load('./FinalSummary_09_10_lowtol.mat');
data = data.FinalSummary;

vel = data(1:size(data, 1)/2, 2);
torque_1 = data(1: size(data, 1)/2, 4);
torque_2 = data(size(data, 1)/2+1: end, 4);

% plot torque vs. speed
figure;
hold on;
xlabel('\tau');
ylabel('\omega');
xlim([0, 1.5]);
ylim([0, 25]);
for i = 1: length(vel)
    plot(torque_1(i), vel(i), 'b*');
    plot(torque_2(i), vel(i), 'ro');
end

% feasible region for motor
p1 = [0, 1]; 
p2 = [0.1571, 1];
p3 = [0.1571, 0];
p4 = [1.5, 0];
p5 = [1.5, 25];
p6 = [0, 25];
X = [p1(1), p2(1), p3(1), p4(1), p5(1), p6(1)];
Y = [p1(2), p2(2), p3(2), p4(2), p5(2), p6(2)];
fill(X, Y, [0.9, 0.8, 0.8]);

%% Pololu 2373
stall_i = 1.6*ones(1, 1); % Unit: A
stall_torq = 12; % Unit: Kg*cm
free_vel = 31; % Unit: RPM
free_i = 0.07; % Unit: A
V = 6; % rated voltage 
[kt, ke ,r, torq, vel] = compute_motor_parameter(stall_i, stall_torq, free_vel, free_i);

tau = 0: 0.01: torq;
omega = -r/(kt*ke)*tau + V/ke;
plot(tau, omega, 'g--'); % motor characteristic cureve

% For motor to work on (0.1571 Nm, 1 rad/s)
omega = 1;
tau = 0.1571;
V = omega*ke + r/kt*tau;
Duty_cycly = V/6;
fprintf("Voltage level is at: %d", V);
fprintf("Duty cycle is: %d", Duty_cycly);
fprintf('\n')

%% Pololu 2386
stall_i = 1.6*ones(1, 1);
stall_torq = 2.4;
free_vel = 210;
free_i = 0.07;
V = 6; % rated voltage 
[kt, ke ,r, torq, vel] = compute_motor_parameter(stall_i, stall_torq, free_vel, free_i);

tau = 0: 0.01: torq;
omega = -r/(kt*ke)*tau + V/ke;
plot(tau, omega, 'r--'); % motor characteristic cureve


% For motor to work on (0.1571 Nm, 1 rad/s)
omega = 1;
tau = 0.1571;
V = omega*ke + r/kt*tau;
Duty_cycly = V/6;
fprintf("Voltage level is at: %d", V);
fprintf("Duty cycle is: %d", Duty_cycly);



    