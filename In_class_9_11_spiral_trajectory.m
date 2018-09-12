% Dynamics in class exercize 9/11/18
% Plotting in cylindrical coordinates 
% 
% Last modified 9/11/18 by hkolano


function Spiral_trajectory
clear all
close all

% Define state variables: 
% z1 = r; z2 = vr; 
% z3 = th; z4 = vth;

% Specify initial conditions. Inital displacements are taken to be zero. 
z1_0 = 0;           
z2_0 = 0;                 
z3_0 = 0;       
z4_0 = 0;		       
Z_0 = [z1_0, z2_0, z3_0, z4_0];

%define time steps
T_span = [0: .05: 5];  

%ODE
% options = odeset('Events', @event_stop);
[t, zout] = ode45(@initial_spring, T_span, Z_0) %, options);

%2D Trajectory Graph
figure (1) 
polarplot(zout(:,1),zout(:,3), 'b-o')
title('Spiral Trajectory')

figure (2)
subplot(2, 1, 1)
plot(t, zout(:,2))
title('Radial Velocity')
subplot(2, 1, 2)
plot(t, zout(:,4))
title('Angular Velocity')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dzdt = initial_spring(T,Z)
    
    % z1 = r; z2 = vr; 
    % z3 = th; z4 = vth;
  
    dz1dt = .2*exp(0.2*Z(3));
    dz2dt = (.2*pi)^2*exp(.2*Z(3));
    dz3dt = pi;
    dz4dt = 0;

    dzdt = [dz1dt;dz2dt;dz3dt;dz4dt];

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










