%% Inverted Pendulum 

clear all, close all, clc, format short 

%% System Inputs

M = 0.292; %[kg]
m = 0.034; %[kg]
Length = 0.151; %[m] %distance to center of rod
b = 0.1; %[-]
I = (1/3)*m*(2*Length)^2; %[kg-m^2] 
g = 9.81; %[m/s2]
r = .0191; %[m] 38.2016mm pitch diameter 
D = 0; %Disturbance

J = 2.12e-5; %[kg-m^2]
Kt = 0.065; %[N-m/A]
R = 1.38; %[Ohm]
L = 2.26e-3; %[H] Inductance
Kv = 0.065; %[V/(rad/s)]
B = 1.21e-5; %[N-m /(rad/s)]

%% Run Simulink file InvertedPendulumControl.slx

% Pendulum Gains
Kp = 0;
Ki = 0; 
Kd = 0; 

% Cart Position Gains
Kpx = 0;
Kix = 0;
Kdx = 0; 

% Cart Velocity Gains
Kpv = 0;
Kiv = 0;
Kdv = 0;

th0 = 0.05; %Assume initial condition of standing up
thd0 = 0; %[rad/s] 0.1222

sim('InvertedPendulumControl.slx')

th = simout(:,1); %theta [deg]
x = simout(:,2); % x [mm]
Torque = simout(:,3); % Force from Motor

%% Plot

% figure
% subplot(3,1,1)
% plot(t,th)
% ylabel('[rad]')
% subplot(3,1,2)
% plot(t,x)
% ylabel('[mm]')
% subplot(3,1,3)
% plot(t,Torque)
% ylabel('[N-m]')
% xlabel('time [s]')

figure
subplot(2,1,1)
plot(t,th)
ylabel('[rad]')
subplot(2,1,2)
plot(t,x)
ylabel('[mm]')


%% Interpolation for smooth animation

time = 0:.01:50; 
x_int = interp1(t,x,time);
th_int = interp1(t,th,time);

% % Any time cart hits limit switches, reset with th = pi, x = 0
% TimeUpright = 0;
% for j = 1:length(time)
%     if x_int(j) >= 241.2
%         x_int(j) = 0;
%         th_int(j) = pi;
%     elseif x_int(j) <= -241.2
%         x_int(j) = 0;
%         th_int(j) = pi;
%     end
%     
%    % determine time upright
% end

%% Animation

% W = 75;  % cart width
% H = 50; % cart height
% y = H/2;
% 
% figure
% for i = 1:length(time)
%         plot([-625 625],[0 0],'k','Linewidth',1) %ground plane
%         hold on
% 
%         P0 = [x_int(i) y];
%         P1 = 2*Length*1000*[sin(th_int(i)) cos(th_int(i))];
% 
%         %cart
%         rectangle('Position',[P0(1)-W/2,P0(2)-H/2,W,H],'Curvature',.5,'FaceColor',[1 0.1 0.1])
% 
%         %pendulum
%         plot([P0(1) P0(1)-P1(1)],[P0(2) P1(2)],'LineWidth',3,'Color',[0 0 0]);
% 
%         xlabel('Track [mm]')
%         ylabel('Vertical [mm]')
%         xlim([-350 350])
%         ylim([-350 350])
%         drawnow
%         hold off
% end
% 
% 
