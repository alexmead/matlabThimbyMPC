% Author: Alex R. Mead
% Date: October 2017
% Description: A practice MPC which will be used as a benchmark to learn
% more about how I can expect these systems to behaive wrt to Thimby.

clear all; close all; clc;



% First, create the SS model
wattage = 0;
thermalMassVolume = 10;

C_1 = (1.66*20)/(1.184*1012*125);
C_2 = (125*0.05)/(1.184*1012*125);
C_3 = (1.66*20)/(2000*880*thermalMassVolume);
C_4 = 1/(1.184*1012*125);
% 
% 
% A = [-C_2 - C_1, C_1; C_3 -C_3];
% B = [C_2 wattage*C_4; 0 0];
% C = [0 0; 0 0];
% D = [0];
% 
% twoStateSimple = ss(A,B,C,D);
% 
% Import the T_out boundary conditions
T_out = csvread('weather.csv');
T_out_F = (9/5).*T_out+32; % convert to F
heater = ones(8760,1);
u = [T_out,heater];
% 
% dt = 3600; % 1 hour time step, in seconds
% t=0:3600:8759*3600;
% %t=0:1:8759;
% 
% % Specify initial conditions, and simulate
% X_0 = [25;25];
% 
% [y,t_sim,x] = lsim(twoStateSimple, u,t,X_0);
% 
% figure(1);hold on;
% plot(t_sim(1:744,1),x(1:744,1),'Marker','*');
% plot(t_sim(1:744,1),x(1:744,2),'Marker','*');
% plot(t_sim(1:744,1),T_out(1:744,1),'Marker','*');
% legend('T_{air}','T_{conc}','T_{out}');
% grid on;
% 
% figure(2);hold on;
% plot(t_sim,x(:,1),'Marker','*');
% plot(t_sim,x(:,2),'Marker','*');
% plot(t_sim,T_out(:,1),'Marker','*');
% legend('T_{air}','T_{conc}','T_{out}');
% grid on;

%%
% Simulate using the discrete model
% 
% % Preallocate memory for the T_air, T_con variables
% T_air = zeros(8760+1,1);
% T_con = zeros(8760+1,1);
% 
% % Specify initial conditions
% T_air(1)=25;
% T_con(1)=25;
% 
% for k=1:8760
%     
%    %  Step forward each state with influence of the boundary conditions
%    T_air(k+1) = T_air(k) + (C_1*(T_con(k)-T_air(k)) + C_2*(T_out(k)-T_air(k)))*3600;
%    T_con(k+1) = T_con(k) + (C_3*(T_air(k)-T_con(k)))*3600;
%     
%     
% end
% 
% 
% % Plot the results
% t_discrete=1:8760+1;
% figure(3); hold on;
% plot(t_discrete,T_air,'Marker','*');
% plot(t_discrete,T_con,'Marker','*');
% plot(1:8760,T_out,'Marker','*');
% legend('T_{air}','T_{con}','T_{out}');
% grid on;

%%
% Simulate using the discrete model, with heating/cooling

% Preallocate memory for the T_air, T_con variables, controller
T_air = zeros(8760+1,1);
T_con = zeros(8760+1,1);
u = zeros(8760+1,1);
setPoint = 20;

% t=0:1:8759;
% T_out = 5*sin(t./100)+20;

% Specify initial conditions
T_air(1)=25;
T_con(1)=25;

% Specs of the problem
addRemove = 40;

for k=1:8760
    
   % Define the heating/cooling control
   dif = setPoint - T_air(k);
   if dif>0
       heatCool = C_4*149776*dif;
       %heatCool=0;
   else
       heatCool = 0;
   end
   
   u(k) = heatCool;
   
   %  Step forward each state with influence of the boundary conditions
   T_air(k+1) = T_air(k) + (C_1*(T_con(k)-T_air(k)) + C_2*(T_out(k)-T_air(k)))*3600 + heatCool;
   T_con(k+1) = T_con(k) + (C_3*(T_air(k)-T_con(k)))*3600;
    
    
end

% Plot the results
t_discrete=1:8760+1;
figure(4); hold on;
plot(t_discrete,T_air,'Marker','*');
plot(t_discrete,T_con,'Marker','*');
plot(1:8760,T_out,'Marker','*');
legend('T_{air}','T_{con}','T_{out}');
grid on;

