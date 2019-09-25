function error_stuff = computeLeaderAgentAverageErrors(agent,data,leader_agent,inter_agent)

error_stuff = struct;

%% Agent 1
try 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Everything that is in agent that uses data.index_form will need to be
% adjusted by the offset
num_el = length(data(1).A.RelTime);
offset = num_el - length(data(1).index);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Get the index of the time we will be analyzing
error_stuff(1).total_time = agent(1).time(agent(1).index_form) - agent(1).time(agent(1).index_form(1)).*ones(length(data(1).index_form),1);
temp_time = error_stuff(1).total_time(end) - 30;
error_stuff(1).analyze_index = find(error_stuff(1).total_time >  temp_time);
error_stuff(1).analyze_time = error_stuff(1).total_time(error_stuff(1).analyze_index);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Average the position errors
avg_agent1_pos = sum(leader_agent(1).R2T_pos_error_1(agent(1).index_form(error_stuff(1).analyze_index),:),1)./length(error_stuff(1).analyze_index);
avg_agent2_pos = sum(leader_agent(1).R2T_pos_error_2(agent(1).index_form(error_stuff(1).analyze_index),:),1)./length(error_stuff(1).analyze_index);
avg_agent3_pos = sum(leader_agent(1).R2T_pos_error_3(agent(1).index_form(error_stuff(1).analyze_index),:),1)./length(error_stuff(1).analyze_index);
error_stuff(1).average_leader_error_pos = (avg_agent1_pos + avg_agent2_pos + avg_agent3_pos)./3;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the standard deviation for the position errors
std_agent1_pos = (leader_agent(1).R2T_pos_error_1(agent(1).index_form(error_stuff(1).analyze_index),:) - avg_agent1_pos.*ones(length(error_stuff(1).analyze_index),1)).^2;
std_agent2_pos = (leader_agent(1).R2T_pos_error_2(agent(1).index_form(error_stuff(1).analyze_index),:) - avg_agent2_pos.*ones(length(error_stuff(1).analyze_index),1)).^2;
std_agent3_pos = (leader_agent(1).R2T_pos_error_3(agent(1).index_form(error_stuff(1).analyze_index),:) - avg_agent3_pos.*ones(length(error_stuff(1).analyze_index),1)).^2;
error_stuff(1).stddev_pos = sqrt((sum(std_agent1_pos,1)./(length(error_stuff(1).analyze_index)-1) + sum(std_agent2_pos,1)./(length(error_stuff(1).analyze_index)-1) + sum(std_agent3_pos,1)./(length(error_stuff(1).analyze_index)-1))./3);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Average the position errors
avg_agent1_vel = sum(leader_agent(1).R2T_vel_error_1(agent(1).index_form(error_stuff(1).analyze_index),:),1)./length(error_stuff(1).analyze_index);
avg_agent2_vel = sum(leader_agent(1).R2T_vel_error_2(agent(1).index_form(error_stuff(1).analyze_index),:),1)./length(error_stuff(1).analyze_index);
avg_agent3_vel = sum(leader_agent(1).R2T_vel_error_3(agent(1).index_form(error_stuff(1).analyze_index),:),1)./length(error_stuff(1).analyze_index);
error_stuff(1).average_leader_error_vel = (avg_agent1_vel + avg_agent2_vel + avg_agent3_vel)./3;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the standard deviation for the position errors
std_agent1_vel = (leader_agent(1).R2T_vel_error_1(agent(1).index_form(error_stuff(1).analyze_index),:) - avg_agent1_vel.*ones(length(error_stuff(1).analyze_index),1)).^2;
std_agent2_vel = (leader_agent(1).R2T_vel_error_2(agent(1).index_form(error_stuff(1).analyze_index),:) - avg_agent2_vel.*ones(length(error_stuff(1).analyze_index),1)).^2;
std_agent3_vel = (leader_agent(1).R2T_vel_error_3(agent(1).index_form(error_stuff(1).analyze_index),:) - avg_agent3_vel.*ones(length(error_stuff(1).analyze_index),1)).^2;
error_stuff(1).stddev_vel = sqrt((sum(std_agent1_vel,1)./(length(error_stuff(1).analyze_index)-1) + sum(std_agent2_vel,1)./(length(error_stuff(1).analyze_index)-1) + sum(std_agent3_vel,1)./(length(error_stuff(1).analyze_index)-1))./3);

catch
    disp('Check vehicle 1 leader-agent stats')
    error_stuff(1).total_time = 0;
    error_stuff(1).analyze_index = 0;
    error_stuff(1).analyze_time = 0;
    error_stuff(1).average_leader_error_pos = 0;
    error_stuff(1).stddev_pos = 0;
    error_stuff(1).average_leader_error_vel = 0;
    error_stuff(1).stddev_vel = 0;
end


%% Agent 2

try 
    
clear temp_time avg_agent1_pos avg_agent2_pos avg_agent3_pos std_agent1_pos std_agent2_pos std_agent3_pos
clear avg_agent1_vel avg_agent2_vel avg_agent3_vel std_agent1_vel std_agent2_vel std_agent3_vel
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Everything that is in agent that uses data.index_form will need to be
% adjusted by the offset
num_el = length(data(2).A.RelTime);
offset = num_el - length(data(2).index);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Get the index of the time we will be analyzing
error_stuff(2).total_time = agent(2).time(agent(2).index_form) - agent(2).time(agent(2).index_form(1)).*ones(length(data(2).index_form),1);
temp_time = error_stuff(2).total_time(end) - 30;
error_stuff(2).analyze_index = find(error_stuff(2).total_time >  temp_time);
error_stuff(2).analyze_time = error_stuff(2).total_time(error_stuff(2).analyze_index);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Average the position errors
avg_agent1_pos = sum(leader_agent(2).R2T_pos_error_1(agent(2).index_form(error_stuff(2).analyze_index),:),1)./length(error_stuff(2).analyze_index);
avg_agent2_pos = sum(leader_agent(2).R2T_pos_error_2(agent(2).index_form(error_stuff(2).analyze_index),:),1)./length(error_stuff(2).analyze_index);
avg_agent3_pos = sum(leader_agent(2).R2T_pos_error_3(agent(2).index_form(error_stuff(2).analyze_index),:),1)./length(error_stuff(2).analyze_index);
error_stuff(2).average_leader_error_pos = (avg_agent1_pos + avg_agent2_pos + avg_agent3_pos)./3;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the standard deviation for the position errors
std_agent1_pos = (leader_agent(2).R2T_pos_error_1(agent(2).index_form(error_stuff(2).analyze_index),:) - avg_agent1_pos.*ones(length(error_stuff(2).analyze_index),1)).^2;
std_agent2_pos = (leader_agent(2).R2T_pos_error_2(agent(2).index_form(error_stuff(2).analyze_index),:) - avg_agent2_pos.*ones(length(error_stuff(2).analyze_index),1)).^2;
std_agent3_pos = (leader_agent(2).R2T_pos_error_3(agent(2).index_form(error_stuff(2).analyze_index),:) - avg_agent3_pos.*ones(length(error_stuff(2).analyze_index),1)).^2;
error_stuff(2).stddev_pos = sqrt((sum(std_agent1_pos,1)./(length(error_stuff(2).analyze_index)-1) + sum(std_agent2_pos,1)./(length(error_stuff(2).analyze_index)-1) + sum(std_agent3_pos,1)./(length(error_stuff(2).analyze_index)-1))./3);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Average the position errors
avg_agent1_vel = sum(leader_agent(2).R2T_vel_error_1(agent(2).index_form(error_stuff(2).analyze_index),:),1)./length(error_stuff(2).analyze_index);
avg_agent2_vel = sum(leader_agent(2).R2T_vel_error_2(agent(2).index_form(error_stuff(2).analyze_index),:),1)./length(error_stuff(2).analyze_index);
avg_agent3_vel = sum(leader_agent(2).R2T_vel_error_3(agent(2).index_form(error_stuff(2).analyze_index),:),1)./length(error_stuff(2).analyze_index);
error_stuff(2).average_leader_error_vel = (avg_agent1_vel + avg_agent2_vel + avg_agent3_vel)./3;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the standard deviation for the position errors
std_agent1_vel = (leader_agent(2).R2T_vel_error_1(agent(2).index_form(error_stuff(2).analyze_index),:) - avg_agent1_vel.*ones(length(error_stuff(2).analyze_index),1)).^2;
std_agent2_vel = (leader_agent(2).R2T_vel_error_2(agent(2).index_form(error_stuff(2).analyze_index),:) - avg_agent2_vel.*ones(length(error_stuff(2).analyze_index),1)).^2;
std_agent3_vel = (leader_agent(2).R2T_vel_error_3(agent(2).index_form(error_stuff(2).analyze_index),:) - avg_agent3_vel.*ones(length(error_stuff(2).analyze_index),1)).^2;
error_stuff(2).stddev_vel = sqrt((sum(std_agent1_vel,1)./(length(error_stuff(2).analyze_index)-1) + sum(std_agent2_vel,1)./(length(error_stuff(2).analyze_index)-1) + sum(std_agent3_vel,1)./(length(error_stuff(2).analyze_index)-1))./3);

catch
    disp('Check vehicle 2 leader-agent stats')
    error_stuff(2).total_time = 0;
    error_stuff(2).analyze_index = 0;
    error_stuff(2).analyze_time = 0;
    error_stuff(2).average_leader_error_pos = 0;
    error_stuff(2).stddev_pos = 0;
    error_stuff(2).average_leader_error_vel = 0;
    error_stuff(2).stddev_vel = 0;
end


%% Agent 3

try 
    
clear temp_time avg_agent1_pos avg_agent2_pos avg_agent3_pos std_agent1_pos std_agent2_pos std_agent3_pos
clear avg_agent1_vel avg_agent2_vel avg_agent3_vel std_agent1_vel std_agent2_vel std_agent3_vel
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Everything that is in agent that uses data.index_form will need to be
% adjusted by the offset
num_el = length(data(3).A.RelTime);
offset = num_el - length(data(3).index);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Get the index of the time we will be analyzing
error_stuff(3).total_time = agent(3).time(agent(3).index_form) - agent(3).time(agent(3).index_form(1)).*ones(length(data(3).index_form),1);
temp_time = error_stuff(3).total_time(end) - 30;
error_stuff(3).analyze_index = find(error_stuff(3).total_time >  temp_time);
error_stuff(3).analyze_time = error_stuff(3).total_time(error_stuff(3).analyze_index);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Average the position errors
avg_agent1_pos = sum(leader_agent(3).R2T_pos_error_1(agent(3).index_form(error_stuff(3).analyze_index),:),1)./length(error_stuff(3).analyze_index);
avg_agent2_pos = sum(leader_agent(3).R2T_pos_error_2(agent(3).index_form(error_stuff(3).analyze_index),:),1)./length(error_stuff(3).analyze_index);
avg_agent3_pos = sum(leader_agent(3).R2T_pos_error_3(agent(3).index_form(error_stuff(3).analyze_index),:),1)./length(error_stuff(3).analyze_index);
error_stuff(3).average_leader_error_pos = (avg_agent1_pos + avg_agent2_pos + avg_agent3_pos)./3;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the standard deviation for the position errors
std_agent1_pos = (leader_agent(3).R2T_pos_error_1(agent(3).index_form(error_stuff(3).analyze_index),:) - avg_agent1_pos.*ones(length(error_stuff(3).analyze_index),1)).^2;
std_agent2_pos = (leader_agent(3).R2T_pos_error_2(agent(3).index_form(error_stuff(3).analyze_index),:) - avg_agent2_pos.*ones(length(error_stuff(3).analyze_index),1)).^2;
std_agent3_pos = (leader_agent(3).R2T_pos_error_3(agent(3).index_form(error_stuff(3).analyze_index),:) - avg_agent3_pos.*ones(length(error_stuff(3).analyze_index),1)).^2;
error_stuff(3).stddev_pos = sqrt((sum(std_agent1_pos,1)./(length(error_stuff(3).analyze_index)-1) + sum(std_agent2_pos,1)./(length(error_stuff(3).analyze_index)-1) + sum(std_agent3_pos,1)./(length(error_stuff(3).analyze_index)-1))./3);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Average the position errors
avg_agent1_vel = sum(leader_agent(3).R2T_vel_error_1(agent(3).index_form(error_stuff(3).analyze_index),:),1)./length(error_stuff(3).analyze_index);
avg_agent2_vel = sum(leader_agent(3).R2T_vel_error_2(agent(3).index_form(error_stuff(3).analyze_index),:),1)./length(error_stuff(3).analyze_index);
avg_agent3_vel = sum(leader_agent(3).R2T_vel_error_3(agent(3).index_form(error_stuff(3).analyze_index),:),1)./length(error_stuff(3).analyze_index);
error_stuff(3).average_leader_error_vel = (avg_agent1_vel + avg_agent2_vel + avg_agent3_vel)./3;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the standard deviation for the position errors
std_agent1_vel = (leader_agent(3).R2T_vel_error_1(agent(3).index_form(error_stuff(3).analyze_index),:) - avg_agent1_vel.*ones(length(error_stuff(3).analyze_index),1)).^2;
std_agent2_vel = (leader_agent(3).R2T_vel_error_2(agent(3).index_form(error_stuff(3).analyze_index),:) - avg_agent2_vel.*ones(length(error_stuff(3).analyze_index),1)).^2;
std_agent3_vel = (leader_agent(3).R2T_vel_error_3(agent(3).index_form(error_stuff(3).analyze_index),:) - avg_agent3_vel.*ones(length(error_stuff(3).analyze_index),1)).^2;
error_stuff(3).stddev_vel = sqrt((sum(std_agent1_vel,1)./(length(error_stuff(3).analyze_index)-1) + sum(std_agent2_vel,1)./(length(error_stuff(3).analyze_index)-1) + sum(std_agent3_vel,1)./(length(error_stuff(3).analyze_index)-1))./3);

catch
    disp('Check vehicle 3 leader-agent stats')
    error_stuff(3).total_time = 0;
    error_stuff(3).analyze_index = 0;
    error_stuff(3).analyze_time = 0;
    error_stuff(3).average_leader_error_pos = 0;
    error_stuff(3).stddev_pos = 0;
    error_stuff(3).average_leader_error_vel = 0;
    error_stuff(3).stddev_vel = 0;
end


end