% Author: Alex R. Mead
% Date: October 2017
% Description: Playing around with MPC from the HEV example in CE295 notes.

clear all; close all; clc;

% Import the boundary conditions: P_dem(k), k=0,...,N
data = csvread('processed_data/UDDS.csv');
t = data(:,1);
MPH = data(:,2);

% Convert MPH to m/s
m_s = MPH.*0.44704;

% Calculate Power Demand
car_mass = 1325; % [kg] Toyota Prius 
KE = 0.5*car_mass.*(m_s.^2); % Kinetic Energy at k=0,...,N
KE = [KE;0]; % Pad with extra 0 for end calculation of power demand

P_dem = zeros(size(t,1),1);
for i=1:size(t,1)
    % Power demain is equal to future state - current state, i.e. how much
    % power is needed to go from now to one step in the future.
    P_dem(i) = KE(i+1) - KE(i);
end

% Convert power from W to kW
P_dem = P_dem./1000;

%plot(P_dem);grid on;




%horizon = 1369;

% Define constraints of the dynamical system
%Battery Dynamics
E_min = 1.296*1000; % [MJ]
E_max = 3.024*1000; % [MJ]
E_0 = 2.16*1000; % [MJ], FYI: 0.6 [kWh]
P_batt_min = -15; % [kW]
P_batt_max = 15; % [kW]
% Engine Dynamics
P_eng_max = 35; % [kW]
alpha = 0.1; % [g/s*kW], the fuel to electricity conversion ratio

dt = 1.0; % [seconds]

% Build a for-loop here to make this a MPC structure.
totalSimulationTime = 1000;
horizon = 60; % 60 second time horizon

E_history = ones(totalSimulationTime+1,1);
E_history(1,1) = E_0;
P_batt_history = ones(totalSimulationTime,1);

for k=1:totalSimulationTime
    k
    
    % Sample current state
    E_0 = E_history(k,1);
    
    % Get current demand forecast
    P_dem_dynamic = P_dem(k:k+60);
    
    % Construct objective function
    f = [-alpha*ones(horizon,1); zeros(horizon+1,1)]';

    % Inequality constaints
    A_1 = [zeros(2,horizon) zeros(2,horizon) [-1; 1]]; % Termination Battery state
    A_2 = [zeros(horizon+1,horizon) -1*eye(horizon+1); zeros(horizon+1,horizon) eye(horizon+1)];% Battery state of charge for the entire horizon
    A_3 = [-1*eye(horizon) zeros(horizon,horizon+1); eye(horizon) zeros(horizon,horizon+1)];
    A_4 = [eye(horizon) zeros(horizon,horizon+1); -1*eye(horizon) zeros(horizon,horizon+1)];
    A = [A_1;A_2;A_3;A_4];

    b_1 = [-0.95*E_0; 1.05*E_0];
    b_2 = [-E_min*ones(horizon+1,1); E_max*ones(horizon+1,1)];
    b_3 = [-P_batt_min*ones(horizon,1); P_batt_max*ones(horizon,1)];
    b_4 = [P_dem_dynamic(1:horizon); P_eng_max*ones(horizon,1)-P_dem_dynamic(1:horizon)];
    b = [b_1;b_2;b_3;b_4];

    % Equality Constraints
    A_eq11 = dt*diag(ones(horizon,1));
    A_eq12 = [-1*eye(horizon)+diag(ones(horizon-1,1),1) [zeros(horizon-1,1) ;1]];
    A_eq21 = zeros(1,horizon);
    A_eq22 = [1 zeros(1,horizon)];
    A_eq = [A_eq11 A_eq12;A_eq21 A_eq22];

    b_eq = [zeros(horizon,1);E_0];

    % Solve the optimization problem
    [x,fval,exitflag,output] = linprog(f,A,b,A_eq,b_eq);
    
    % Using current control input, P_batt, step state forward
    E_history(k+1,1) = E_history(k,1) - x(1)*dt;
    
    % Store control history
    P_batt_history(k,1) = x(1);

end
