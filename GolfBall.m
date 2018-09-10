% Dynamics Assignment 1: Golf Ball
%
% Simulation of golf ball with an ititial velocity and position
% Includes air speed and drag
%
% last modified 9/5/18 HKolano
%
function golf_ball_fun
clear all
close all

% Define system parameters

g = 9.81;               % gravitational acceleration in m/s^2
D = .0427;              % diameter of a golf ball, in m
rho = 1.29;             % density of the air, kg/m3
Cd = .25;               % coefficient of drag
m = .0459;              % mass in kg
drag_c = -.5*rho*Cd*pi*D^2/4;

% Define state variables: 
% z1 = x_i; z2 = x_j; z3 = x_k;
% z4 = v_i; z5 = v_j; z6 = v_k;

% Specify initial conditions. Inital displacements are taken to be zero. 
z1_0 = 0;           
z2_0 = 10;       %initial x velocity           
z3_0 = 0;       
z4_0 = 0;		%initial y velocity
z5_0 = 0;
z6_0 = 10;       %initial z velocity
Z_0 = [z1_0, z2_0, z3_0, z4_0, z5_0, z6_0];

%define the wind
w_x = -10;
w_y = 2;
w_z = 0;

%define time steps
T_span = [0: 0.1: 25];  

%keep track of acceleration
acc_x = [];
acc_y = [];
acc_z = [];

%ODE
options = odeset('Events', @event_stop);
[t, zout] = ode45(@initial_spring, T_span, Z_0, options);

acc_x = acc_x(1:length(t))';
acc_y = acc_y(1:length(t))';
acc_z = acc_z(1:length(t))';

%Start the graph
figure (1) 
plot3(zout(:,1),zout(:,3), zout(:,5))
axis equal
title('Displacements')
zlabel('z Displ (m)')
ylabel('y Displ (m)')
xlabel('x Displ (m)')
legend('ball trajectory')

figure(2)
subplot(3,1,1), plot(t,zout(:,1)) 
hold on
plot(t, zout(:,3)), plot(t, zout(:,5))
ylabel('Displacement (m)')
legend('x-dir', 'y-dir', 'z-dir')

subplot(3,1,2), plot (t, zout(:,2))
hold on
plot(t, zout(:,4)), plot(t, zout(:,6))
ylabel('Velocity (m/s)')
legend('x-dir', 'y-dir', 'z-dir')

subplot(3,1,3), plot(t, acc_x)
hold on
plot(t, acc_y), plot(t, acc_z)
ylabel('Acceleration (m/s2)')
legend('x-dir', 'y-dir', 'z-dir')
%
xlabel('Time (sec)')

%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dzdt = initial_spring(T,Z)
    % Magnitude of ball's velocity relative to air
    V = sqrt((Z(2)-w_x)^2 + (Z(4)-w_y)^2 + (Z(6)-w_z)^2);
 
    % z1 = x_i; z2 = x_j; z3 = x_k;
    % z4 = v_i; z5 = v_j; z6 = v_k;
    dz1dt = Z(2);
    dz2dt = drag_c*V*(Z(2)-w_x)/m;
    dz3dt = Z(4);
    dz4dt = drag_c*V*(Z(4)-w_y)/m;
    dz5dt = Z(6);
    dz6dt = drag_c*V*(Z(6)-w_z)/m-g;

    acc_x = [acc_x, dz2dt]; acc_y = [acc_y, dz4dt]; acc_z = [acc_z, dz6dt];
    dzdt = [dz1dt;dz2dt;dz3dt;dz4dt;dz5dt;dz6dt];

end
      
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [eventvalue,stopthecalc, eventdir] = event_stop(T,Z)
     
        % stop when TopMass gets to L off the ground (spring unstretched)
        eventvalue  =  Z(5);    %  ‘Events’ are detected when eventvalue=0
        stopthecalc =  1;       %  Stop if event occurs
        eventdir    =  -1;       %  Detect only events with dydt<0
end
      
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
end










