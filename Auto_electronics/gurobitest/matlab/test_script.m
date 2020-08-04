
%  stall_i = 1.5*ones(11, 1);
%  stall_torq = [0.09, 0.17, 0.45, 0.74, 1.1, 1.6, 2.0, 2.8, 3.2, 3.4, 11]';
%  free_vel = [6000, 3000, 1100, 650, 430, 330, 220, 160, 130, 110 33]';
%  free_i = 0.1;

% stall_i = 0.75*ones(11, 1);
% stall_torq = [0.09, 0.17, 0.39, 0.67, 1.0, 1.3, 1.8, 2.5, 3.0, 3.3, 10]';
% free_vel = [6000, 3000, 1100, 650, 450, 330, 220, 160, 130, 110 35]';
% free_i = 0.06;

% stall_i = 0.8*ones(2, 1);
% stall_torq = [1.4, 2.2]';
% free_vel = [120, 80]';
% free_i = 0.08;

%% Pololu 2373 specs 
%   stall_i = 1.6*ones(1, 1); % Unit: A
%   stall_torq = 12; % Unit: Kg*cm
%   free_vel = 31; % Unit: RPM
%   free_i = 0.07; % Unit: A

%% Pololu 2216 specs
%    stall_i = 1.6*ones(1, 1);
%    stall_torq = 3.0;
%    free_vel = 150;
%    free_i = 0.07;
 
%% Pololu 993
%    stall_i = 0.36*ones(1, 1);
%    stall_torq = 0.29;
%    free_vel = 450;
%    free_i = 0.02;

%% Pololu 994
%    stall_i = 1.6*ones(1, 1);
%    stall_torq = 4;
%    free_vel = 100;
%    free_i = 0.07;

%% LS-0009AF
% v = 6;
% stall_i = 1;
% stall_torq = 0.14709975;
% free_vel = 10.4720;
% free_i = 0.15;

%% Pololu 3039
% v = 12;
% stall_i = 0.75;
% stall_torq = 0.065704555;
% free_vel = 68.06784075;
% free_i = 0.06;

%% Pololu 3041
% v = 12;
% stall_i = 0.75;
% stall_torq = 0.12748645;
% free_vel = 34.55751915;
% free_i = 0.06;

%% Pololu 3044
% v = 12;
% stall_i = 0.75; 
% stall_torq = 0.2941995;
% free_vel = 13.613568149999999;
% free_i = 0.06;

%% Pololu 3062
% v = 6;
% stall_i = 1.5;
% stall_torq = 0.044129925;
% free_vel = 115.19173049999999;
% free_i = 0.1;

%% Pololu 3064
v = 6;
stall_i = 1.5;
stall_torq = 0.10787315000000001;
free_vel = 45.02949465;
free_i = 0.1;

%% Pololu 3066
% v = 6;
% stall_i = 1.5; 
% stall_torq = 0.196133;
% free_vel = 23.0383461;
% free_i = 0.1;


%% Pololu 4788
% v = 12;
% stall_i = 0.75;
% stall_torq = 0.024516625;
% free_vel = 230.38346099999998;
% free_i = 0.06;

%% Pololu 4796
% v = 6;
% stall_i = 1.5;
% stall_torq = 0.4903325;
% free_vel = 8.901179175;
% free_i = 0.1;

%% LS-0009AF
% v = 6;
% stall_i = 1;
% stall_torq = 0.14709975;
% free_vel = 10.4720;
% free_i = 0.15;

%% LS-S8220
% v = 6;
% stall_i = 1;
% stall_torq = 0.14709975;
% free_vel = 10.4720;
% free_i = 0.14;

%% ROB-11965
% v = 6;
% stall_i = 1.1;
% stall_torq = 0.588399;
% free_vel = 6.5450;
% free_i = 0.19;

%% SER0037
% v = 6;
% stall_i = 0.5;
% stall_torq = 0.0588399;
% free_vel = 14.9600;
% free_i = 0.1; 


% torque unit: Nm, velocity unit: rad/s
% [kt, ke ,r, torq, vel] = compute_motor_parameter(stall_i, stall_torq, free_vel, free_i, v);

kt = stall_torq./stall_i;
r = v./stall_i;
ke = (-r.*free_i+v)./free_vel; 