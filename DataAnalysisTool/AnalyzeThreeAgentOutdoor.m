function [agent,leader_agent,inter_agent] = AnalyzeThreeAgentOutdoor(data,target_vectors)


% target_vectors = [0.9 0.43 -0.2; -0.9 0.43 -0.2; 0.0 -0.9 -0.2];

agent = struct;
leader_agent = struct;
inter_agent = struct;

A1 = data(1).A;
A2 = data(2).A;
A3 = data(3).A;


%% Set agent properties
% Agent 1
try
%     index1 = find(A1.v1_flightSequence > 0);
    index1 = data(1).index;
    agent(1).time = data(1).A.RelTime(index1);
    
    % Set the index_form parameter
    agent(1).flight_sequence = data(1).A.v1_flightSequence(index1);
    agent(1).index_virt = find(agent(1).flight_sequence == 5);
    agent(1).index_form = find(agent(1).flight_sequence == 6);
    
    
    % Get positiong velocity and input
    agent(1).pos_1 = [data(1).A.v1_lat(index1),data(1).A.v1_lon(index1),data(1).A.v1_zPos(index1)];
    agent(1).vel_1 = [data(1).A.v1_xVel(index1),data(1).A.v1_yVel(index1),data(1).A.v1_zVel(index1)];
    
    % Smooth the agent velocity and estimate the actual acceleration using
    % MATLAB 'sgolayfilt' function
    agent(1).vel_sgo = [sgolayfilt(data(1).A.v1_xVel,2,11),sgolayfilt(data(1).A.v1_yVel,2,11), sgolayfilt(data(1).A.v1_zVel,2,11)];
    agent(1).acc_sgo = [[0,0,0];diff(agent(1).vel_sgo)./[diff(data(1).A.RelTime) diff(data(1).A.RelTime) diff(data(1).A.RelTime)]];
    
    % Get the acceleration input from the flights
    agent(1).ctrl = [data(1).A.v1_ux(index1), data(1).A.v1_uy(index1), data(1).A.v1_uz(index1)];
    
    % Set the throttle parameters
    agent(1).throttle = data(1).A.v1_tPWM_des;
    
    % Set the battery parameters
    agent(1).battery = data(1).A.v1_battVolt;
    
    % Get the leader trajectory
    agent(1).leader_pos = [A1.v1_lead_lat(index1),A1.v1_lead_lon(index1),A1.v1_leadZPos(index1)];
    agent(1).leader_vel = [data(1).A.v1_leadXVel(index1),data(1).A.v1_leadYVel(index1),data(1).A.v1_leadZVel(index1)];
    
    % Get the Relative to Target Values
    [agent(1).R2T,~] = computeR2T(index1,data(1).A,target_vectors);
    
    % Get the leader-agent errors
    leader_agent(1).pos_error_1 = agent(1).pos_1 - [data(1).A.v1_lead_lat(index1),data(1).A.v1_lead_lon(index1),data(1).A.v1_leadZPos(index1)];
    leader_agent(1).vel_error_1 = agent(1).vel_1 - [data(1).A.v1_leadXVel(index1),data(1).A.v1_leadYVel(index1),data(1).A.v1_leadZVel(index1)];
    leader_agent(1).R2T_pos_error_1 = agent(1).pos_1 - agent(1).R2T.R2T_1';
    leader_agent(1).R2T_vel_error_1 = agent(1).vel_1 - agent(1).R2T.R2T_dot_1';
    try
        agent(1).pos_2 = [A1.v2_xPos(index1),A1.v2_yPos(index1),A1.v2_zPos(index1)];
        agent(1).vel_2 = [A1.v2_xVel(index1),A1.v2_yVel(index1),A1.v2_zVel(index1)];
        leader_agent(1).pos_error_2 = agent(1).pos_2 - [A1.v1_leadXPos(index1),A1.v1_leadYPos(index1),A1.v1_leadZPos(index1)];
        leader_agent(1).vel_error_2 = agent(1).vel_2 - [A1.v1_leadXVel(index1),A1.v1_leadYVel(index1),A1.v1_leadZVel(index1)];
        leader_agent(1).R2T_pos_error_2 = agent(1).pos_2 - agent(1).R2T.R2T_2';
        leader_agent(1).R2T_vel_error_2 = agent(1).vel_2 - agent(1).R2T.R2T_dot_2';
        inter_agent(1).R2T_pos_error_12 = leader_agent(1).R2T_pos_error_1 - leader_agent(1).R2T_pos_error_2;
        inter_agent(1).R2T_vel_error_12 = leader_agent(1).R2T_vel_error_1 - leader_agent(1).R2T_vel_error_2;
    catch
        disp('Problem in vehicle 1 leader-agent position error: 2')
    end
    
    try
        agent(1).pos_3 = [A1.v3_xPos(index1),A1.v3_yPos(index1),A1.v3_zPos(index1)];
        agent(1).vel_3 = [A1.v3_xVel(index1),A1.v3_yVel(index1),A1.v3_zVel(index1)];
        leader_agent(1).pos_error_3 = agent(1).pos_3 - [A1.v1_leadXPos(index1),A1.v1_leadYPos(index1),A1.v1_leadZPos(index1)];
        leader_agent(1).vel_error_3 = agent(1).vel_3 - [A1.v1_leadXVel(index1),A1.v1_leadYVel(index1),A1.v1_leadZVel(index1)];
        leader_agent(1).R2T_pos_error_3 = agent(1).pos_3 - agent(1).R2T.R2T_3';
        leader_agent(1).R2T_vel_error_3 = agent(1).vel_3 - agent(1).R2T.R2T_dot_3';
        inter_agent(1).R2T_pos_error_13 = leader_agent(1).R2T_pos_error_1 - leader_agent(1).R2T_pos_error_3;
        inter_agent(1).R2T_vel_error_13 = leader_agent(1).R2T_vel_error_1 - leader_agent(1).R2T_vel_error_3;
    catch
        disp('Problem in vehicle 1 leader-agent position error: 3')
    end
    
catch
    disp('No data from vehicle 1.')
end

% Agent 2
try
%     index2 = find(A2.v2_flightSequence > 0);
    index2 = data(2).index;
    agent(2).time = A2.RelTime(index2);
    
    
    % Set the index_form parameter
    agent(2).flight_sequence = A2.v2_flightSequence(index2);
    agent(2).index_virt = find(agent(2).flight_sequence == 5);
    agent(2).index_form = find(agent(2).flight_sequence == 6);
    
        
    % Get positiong velocity and input
    agent(2).pos_2 = [A2.v2_xPos(index2),A2.v2_yPos(index2),A2.v2_zPos(index2)];
    agent(2).vel_2 = [A2.v2_xVel(index2),A2.v2_yVel(index2),A2.v2_zVel(index2)];
    
    % Smooth the agent velocity and estimate the actual acceleration using
    % MATLAB 'sgolayfilt' function
    agent(2).vel_sgo = [sgolayfilt(A2.v2_xVel,2,11),sgolayfilt(A2.v2_yVel,2,11), sgolayfilt(A2.v2_zVel,2,11)];
    agent(2).acc_sgo = [[0,0,0];diff(agent(2).vel_sgo)./[diff(A2.RelTime) diff(A2.RelTime) diff(A2.RelTime)]];
    
    % Get the acceleration input from the flights
    agent(2).ctrl = [A2.v2_ux(index2), A2.v2_uy(index2), A2.v2_uz(index2)];
    
    % Set the throttle parameters
    agent(2).throttle = A2.v2_tPWM_des;
    
    % Set the battery parameters
    agent(2).battery = A2.v2_battVolt;
    
    % Get the R2T Values
    [agent(2).R2T,~] = computeR2T(index2,A2,target_vectors);
    
    % Get the leader-agent errors
    leader_agent(2).pos_error_2 = agent(2).pos_2 - [A2.v2_leadXPos(index2),A2.v2_leadYPos(index2),A2.v2_leadZPos(index2)];
    leader_agent(2).vel_error_2 = agent(2).vel_2 - [A2.v2_leadXVel(index2),A2.v2_leadYVel(index2),A2.v2_leadZVel(index2)];
    leader_agent(2).R2T_pos_error_2 = agent(2).pos_2 - agent(2).R2T.R2T_2';
    leader_agent(2).R2T_vel_error_2 = agent(2).vel_2 - agent(2).R2T.R2T_dot_2';
    try
        agent(2).pos_1 = [A2.v1_xPos(index2),A2.v1_yPos(index2),A2.v1_zPos(index2)];
        agent(2).vel_1 = [A2.v1_xVel(index2),A2.v1_yVel(index2),A2.v1_zVel(index2)];
        leader_agent(2).pos_error_1 = agent(2).pos_1 - [A2.v2_leadXPos(index2),A2.v2_leadYPos(index2),A2.v2_leadZPos(index2)];
        leader_agent(2).vel_error_1 = agent(2).vel_1 - [A2.v2_leadXVel(index2),A2.v2_leadYVel(index2),A2.v2_leadZVel(index2)];
        leader_agent(2).R2T_pos_error_1 = agent(2).pos_1 - agent(2).R2T.R2T_1';
        leader_agent(2).R2T_vel_error_1 = agent(2).vel_1 - agent(2).R2T.R2T_dot_1';
        inter_agent(2).R2T_pos_error_21 = leader_agent(2).R2T_pos_error_2 - leader_agent(2).R2T_pos_error_1;
        inter_agent(2).R2T_vel_error_21 = leader_agent(2).R2T_vel_error_2 - leader_agent(2).R2T_vel_error_1;
    catch
        disp('Problem in vehicle 2 leader-agent position error: 1')
    end
    
    try
        agent(2).pos_3 = [A2.v3_xPos(index2),A2.v3_yPos(index2),A2.v3_zPos(index2)];
        agent(2).vel_3 = [A2.v3_xVel(index2),A2.v3_yVel(index2),A2.v3_zVel(index2)];
        leader_agent(2).pos_error_3 = agent(2).pos_3 - [A2.v2_leadXPos(index2),A2.v2_leadYPos(index2),A2.v2_leadZPos(index2)];
        leader_agent(2).vel_error_3 = agent(2).vel_3 - [A2.v2_leadXVel(index2),A2.v2_leadYVel(index2),A2.v2_leadZVel(index2)];
        leader_agent(2).R2T_pos_error_3 = agent(2).pos_3 - agent(2).R2T.R2T_3';
        leader_agent(2).R2T_vel_error_3 = agent(2).vel_3 - agent(2).R2T.R2T_dot_3';
        inter_agent(2).R2T_pos_error_23 = leader_agent(2).R2T_pos_error_2 - leader_agent(2).R2T_pos_error_3;
        inter_agent(2).R2T_vel_error_23 = leader_agent(2).R2T_vel_error_2 - leader_agent(2).R2T_vel_error_3;
    catch
        disp('Problem in vehicle 2 leader-agent position error: 3')
    end
catch
    disp('No data from vehicle 2.')
end

% Agent 3
try
%     index3 = find(A3.v3_flightSequence > 0);
    index3 = data(3).index;
    agent(3).time = A3.RelTime(index3);
    
    
    % Set the index_form parameter
    agent(3).flight_sequence = A3.v3_flightSequence(index3);
    agent(3).index_virt = find(agent(3).flight_sequence == 5);
    agent(3).index_form = find(agent(3).flight_sequence == 6);
    
    
    % Get positiong velocity and input
    agent(3).pos_3 = [A3.v3_xPos(index3),A3.v3_yPos(index3),A3.v3_zPos(index3)];
    agent(3).vel_3 = [A3.v3_xVel(index3),A3.v3_yVel(index3),A3.v3_zVel(index3)];
    
    % Smooth the agent velocity and estimate the actual acceleration using
    % MATLAB 'sgolayfilt' function
    agent(3).vel_sgo = [sgolayfilt(A3.v3_xVel,2,11),sgolayfilt(A3.v3_yVel,2,11), sgolayfilt(A3.v3_zVel,2,11)];
    agent(3).acc_sgo = [[0,0,0];diff(agent(3).vel_sgo)./[diff(A3.RelTime) diff(A3.RelTime) diff(A3.RelTime)]];
    
    % Get the acceleration input from the flights
    agent(3).ctrl = [A3.v3_ux(index3), A3.v3_uy(index3), A3.v3_uz(index3)];
    
    % Set the throttle parameters
    agent(3).throttle = A3.v3_tPWM_des;
    
    % Set the battery parameters
    agent(3).battery = A3.v3_battVolt;
    
    % Get the R2T Values
    [agent(3).R2T,~] = computeR2T(index3,A3,target_vectors);
    
    % Get the leader-agent errors
    leader_agent(3).pos_error_3 = agent(3).pos_3 - [A3.v3_leadXPos(index3),A3.v3_leadYPos(index3),A3.v3_leadZPos(index3)];
    leader_agent(3).vel_error_3 = agent(3).vel_3 - [A3.v3_leadXVel(index3),A3.v3_leadYVel(index3),A3.v3_leadZVel(index3)];
    leader_agent(3).R2T_pos_error_3 = agent(3).pos_3 - agent(3).R2T.R2T_3';
    leader_agent(3).R2T_vel_error_3 = agent(3).vel_3 - agent(3).R2T.R2T_dot_3';
    try
        agent(3).pos_1 = [A3.v1_xPos(index3),A3.v1_yPos(index3),A3.v1_zPos(index3)];
        agent(3).vel_1 = [A3.v1_xVel(index3),A3.v1_yVel(index3),A3.v1_zVel(index3)];
        leader_agent(3).pos_error_1 = agent(3).pos_1 - [A3.v3_leadXPos(index3),A3.v3_leadYPos(index3),A3.v3_leadZPos(index3)];
        leader_agent(3).vel_error_1 = agent(3).vel_1 - [A3.v3_leadXVel(index3),A3.v3_leadYVel(index3),A3.v3_leadZVel(index3)];
        leader_agent(3).R2T_pos_error_1 = agent(3).pos_1 - agent(3).R2T.R2T_1';
        leader_agent(3).R2T_vel_error_1 = agent(3).vel_1 - agent(3).R2T.R2T_dot_1';
        inter_agent(3).R2T_pos_error_31 = leader_agent(3).R2T_pos_error_3 - leader_agent(3).R2T_pos_error_1;
        inter_agent(3).R2T_vel_error_31 = leader_agent(3).R2T_vel_error_3 - leader_agent(3).R2T_vel_error_1;
    catch
        disp('Problem in vehicle 3 leader-agent position error: 1')
    end
    
    try
        agent(3).pos_2 = [A3.v2_xPos(index3),A3.v2_yPos(index3),A3.v2_zPos(index3)];
        agent(3).vel_2 = [A3.v2_xVel(index3),A3.v2_yVel(index3),A3.v2_zVel(index3)];
        leader_agent(3).pos_error_2 = agent(3).pos_2 - [A3.v3_leadXPos(index3),A3.v3_leadYPos(index3),A3.v3_leadZPos(index3)];
        leader_agent(3).vel_error_2 = agent(3).vel_2 - [A3.v3_leadXVel(index3),A3.v3_leadYVel(index3),A3.v3_leadZVel(index3)];
        leader_agent(3).R2T_pos_error_2 = agent(3).pos_2 - agent(3).R2T.R2T_2';
        leader_agent(3).R2T_vel_error_2 = agent(3).vel_2 - agent(3).R2T.R2T_dot_2';
        inter_agent(3).R2T_pos_error_32 = leader_agent(3).R2T_pos_error_3 - leader_agent(3).R2T_pos_error_2;
        inter_agent(3).R2T_vel_error_32 = leader_agent(3).R2T_vel_error_3 - leader_agent(3).R2T_vel_error_2;
    catch
        disp('Problem in vehicle 3 leader-agent position error: 2')
    end
catch
    disp('No data from vehicle 3.')
end


% % Plot the PWM and Battery Voltage
% figure
% subplot(2,1,1)
% plot(agent(1).time(index1),agent(1).throttle(index1))
% hold on
% plot(agent(2).time(index2),agent(2).throttle(index2))
% plot(agent(3).time(index3),agent(3).throttle(index3))
% hold off
% xlim([agent(1).time(1) agent(1).time(end)])
% grid on
% 
% subplot(2,1,2)
% plot(agent(1).time(index1),agent(1).battery(index1))
% hold on
% plot(agent(2).time(index2),agent(2).battery(index2))
% plot(agent(3).time(index3),agent(3).battery(index3))
% hold off
% xlim([agent(1).time(1) agent(1).time(end)])
% grid on

















% Sanity Check to make sure the agents think everyone is where we think they are 
% 
% figure 
% subplot(3,1,1)
% plot(A1.RelTime(index1),A1.v1_xPos(index1))
% hold on 
% plot(A3.RelTime(index3),A3.v1_xPos(index3))
% plot(A2.RelTime(index2),A2.v1_xPos(index2))
% hold off
% grid on
% 
% subplot(3,1,2)
% plot(A1.RelTime(index1),A1.v2_xPos(index1))
% hold on
% plot(A3.RelTime(index3),A3.v2_xPos(index3))
% plot(A2.RelTime(index2),A2.v2_xPos(index2))
% hold off
% grid on
% 
% subplot(3,1,3)
% plot(A1.RelTime(index1),A1.v3_xPos(index1))
% hold on
% plot(A3.RelTime(index3),A3.v3_xPos(index3))
% plot(A2.RelTime(index2),A2.v3_xPos(index2))
% hold off
% grid on







end