% 2massSpringFunction.m 
%
% Simulation of a vertical spring between two masses
% Gravity acts down in the vertical (-y) direction.
% No energy loss (e.g., damping, drag) 
%
% last modified 9/4/18 HKolano
%
function springsfunction
clear all
close all

% Define system parameters

g = 9.81;              % gravitational acceleration in m/s^2
mTop = .02;             % mass in kg
mBottom = .01;
k = 9810;                 %spring constant
L_0 = .05;               %unstretched length of spring, m
d = .005;                 %distance compressed

% Define state variables: 
%  z1 = y_b, z2 = dy_b/dt, z3 = y_t, z4 = dy_t/dt

% Specify initial velocity. Inital displacements are taken to be zero. 
z1_0 = 0;           %Bottom starts on the ground
z2_0 = 0;           
z3_0 = L_0-d;       %Top starts some distance away from the ground
z4_0 = 0;		
Z_0 = [z1_0, z2_0, z3_0, z4_0];

%define time steps
T_span = [0: 0.0001: 2];  

%first ODE, getting off the ground
options = odeset('Events', @event_stop);
[t, zout] = ode45(@initial_spring, T_span, Z_0, options);

%Start the graph
figure (1) 
plot(t, zout(:,3))
hold
plot(t, zout(:,1))

%define new starting point
t_start = t(end);
Z_out = [zout(end, 1), zout(end,2), zout(end,3), zout(end,4)];
T_span2 = [t_start:0.0001:2];

%plot(x_eqn, y_eqn, 'r+')
title('Trajectory in Time')
ylabel('Vert Displ (m)')
xlabel('Time (s)')
 
options2 = odeset('Events', @event_stop2);
[t2, zout2] = ode45(@air_spring, T_span2, Z_out, options2);
plot(t2, zout2(:,3));
plot(t2, zout2(:,1));

%Normal Force
Norm1 = -(zout(:,3)-zout(:,1)-L_0)*k

%kinetic, potential, and total energy 
all_vs_bottom = cat(1, zout(:,2), zout2(:,2));
all_vs_top = cat(1, zout(:,4), zout2(:,4));
all_pos_bottom = cat(1, zout(:,1), zout2(:,1));
all_pos_top = cat(1, zout(:,3), zout2(:,3));
all_ts = cat(1, t, t2);

KE_top = 1/2.*mTop.*all_vs_top.^2;
KE_bottom = 1/2.*mBottom.*all_vs_bottom.^2;
PE_top =  mTop * g * all_pos_top;
PE_bottom = mBottom * g * all_pos_bottom;
E_of_spring = 0.5*((all_pos_top - all_pos_bottom)-L_0).^2*k;
Etotal = KE_top + KE_bottom + PE_top + PE_bottom + E_of_spring;   

figure (2)
plot(all_ts, KE_top)
hold
plot(all_ts, KE_bottom)
plot(all_ts, PE_top)
plot(all_ts, PE_bottom)
plot(all_ts, E_of_spring)
plot(all_ts, Etotal)
title('Kinetic, Potential Energies')
xlabel('Time (s)')
ylabel('Energy (J)')
legend('KE_top','KE_bottom', 'PE_top', 'PE_bottom', 'spring', 'total')

%Plot normal force over time
figure (3)
plot(t, Norm1, 'g')
title('Normal Force over Time')
xlabel('Time (s)')
ylabel('Force (N)')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dzdt = initial_spring(T,Z)

    spring_force = k*(Z(3)-Z(1)-L_0);
    % Z(1)= z1=y_b, Z(2)= z2=dy_b/dt, Z(3)= z3=y_t, Z(4)= z4=dy_t/dt
    dz1dt = Z(2);
    dz2dt = 0;
    dz3dt = Z(4);
    dz4dt = -spring_force/mTop-g;
    
    
    dzdt = [dz1dt;dz2dt;dz3dt;dz4dt];

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function dzdt = air_spring(T,Z)
    % Z(1)= z1=y_b, Z(2)= z2=dy_b/dt, Z(3)= z3=y_t, Z(4)= z4=dy_t/dt
    spring_force = k*(Z(3)-Z(1)-L_0);
    
    dz1dt = Z(2); 
    dz2dt = spring_force/mBottom-g;
    dz3dt = Z(4);
    dz4dt = -spring_force/mTop-g;
    
    dzdt = [dz1dt;dz2dt;dz3dt;dz4dt];
end
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [eventvalue,stopthecalc, eventdir] = event_stop(T,Z)
     
        % stop when TopMass gets to L off the ground (spring unstretched)
        eventvalue  =  (Z(3)-Z(1)-L_0)*k - mBottom*g;    %  ‘Events’ are detected when eventvalue=0
        stopthecalc =  1;       %  Stop if event occurs
        eventdir    =  1;       %  Detect only events with dydt<0
end
      
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [eventvalue,stopthecalc, eventdir] = event_stop2(T,Z)
     
        % stop when the lower mass touches the ground
        eventvalue  =  Z(1);    %  ‘Events’ are detected when eventvalue=0
        stopthecalc =  1;       %  Stop if event occurs
        eventdir    =  -1;       %  Detect only events with dydt<0
end
      
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
end










