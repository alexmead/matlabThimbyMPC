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

%
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

%
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
       heatCool = 0.95*C_4*149776*dif;
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

%%

% Attempting to set up a MPC wrt the heatCool minimization. I will only for
% a heating period for simplicity the first time around.

testPeriod = 8760/6; % Winter months: Jan, Feb
N=48; % Two day control horizon
dt = 3600; % seconds in one hour

% Preallocate memory for states and control
T_air_hist = zeros(testPeriod+1,1);
T_con_hist = zeros(testPeriod+1,1);
u_hist = zeros(testPeriod+1,1);

% Modify T_out for end hours of MPC
T_out_ext = fliplr(T_out(end-49:end)');
T_out = [T_out;T_out_ext'];

% Specify initial conditions
T_air_hist(1) = 25;
T_con_hist(1) = 25;
setPoint = 20;
heaterPower = 2;

for k=1:testPeriod
    k
    % Gather the T_out and initial condition data 
    T_out_hor = T_out(k:k+N);
    T_con_0 = T_con_hist(k);
    
    % Construct f vector
    %f=[-ones(1,N) zeros(1,N) zeros(1,N)];
    
    % H matrix
    H = [eye(N) zeros(N,N) zeros(N,N); zeros(N,N) zeros(N,N) zeros(N,N); zeros(N,N) zeros(N,N) zeros(N,N)];
    
    % f vector
    f = [-44*ones(N,1);zeros(2*N,1)];
    
    
    % Equality Constraints
    % Air dynamics
    T_air_term = 1-heaterPower -C_1*dt - C_2*dt;
    A_eq_T_air = T_air_term*eye(N) + diag(-1*ones(N-1,1),1);
    A_eq_T_con = C_2*dt*eye(N);
    A_eq_T_out = -C_2*dt*eye(N);
    A_eq_air = [A_eq_T_air A_eq_T_con A_eq_T_out];
    b_eq_air = zeros(N,1);
    % Concrete dynamics
    A_eq_con1 = C_3*dt*eye(N);
    T_con_term = 1-C_3*dt;
    A_eq_con2 = T_con_term*eye(N) + diag(-1*ones(N-1,1),1);
    A_eq_con = [A_eq_con1 A_eq_con2 zeros(N,N)];
    b_eq_con = zeros(N,1);
    
    A_eq = [A_eq_air; A_eq_con];
    b_eq = [b_eq_air; b_eq_con];
    
    % Inequality Constraints
    A_T_con_term = [zeros(1,N) zeros(1,N-1) -1 zeros(1,N);zeros(1,N) zeros(1,N-1) 1 zeros(1,N)];
    b_T_con_term = [-0.95*T_con_0; 1.05*T_con_0];
    
    A_T_air_constraint = [-1*eye(N) zeros(N) zeros(N);eye(N) zeros(N) zeros(N)];
    b_T_air_constraint = [-14*ones(N,1);22*ones(N,1)];
    
    A = [A_T_con_term; A_T_air_constraint];
    b = [b_T_con_term; b_T_air_constraint];
    
    % Solve optimization
    %[x,fval,exitflag,output] = linprog(f,A,b,A_eq,b_eq);
    [x,fval,exitflag,output] = quadprog(H,f,A,b,A_eq,b_eq);
    
    % Update states, record values
    
    u_hist(k) = 0.95*C_4*149776*(setPoint - T_air_hist(k));
    T_air_hist(k+1) = T_air_hist(k) + (C_1*(T_con_hist(k)-T_air_hist(k)) + C_2*(T_out_hor(1)-T_air_hist(k)))*dt + u_hist(k);
    T_con_hist(k+1) = T_con_hist(k) + C_3*(T_air_hist(k)-T_con_hist(k))*dt;





end





























