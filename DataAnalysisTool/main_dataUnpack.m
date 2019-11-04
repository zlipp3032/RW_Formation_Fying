

clear
clc
close all


disp('Main data analysis tool for 3DR-SOLO formation flight tests.')

% Unpacking data stuff
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Prescribe path to the data files
path = '/Users/Zack/Documents/Experiments/Outdoor/Data/';
test = '';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Pick the file strings
data = struct;
data(1).file = '2019_09_07__13_54_00_log_v1';
data(2).file = '2019_09_07__06_30_43_log_v2';
data(3).file = '2019_09_06__20_02_54_log_v3';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Processing Data stuff
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the desired distance in the leader frame
% target_vectors = [1.5 0.43 -0.2; -1.5 0.43 -0.2; 0.0 -1.2 -0.2]; 
% target_vectors = [0.75 0.43 -0.2; -0.75 0.43 -0.2; 0.0 -0.87 -0.2]; 
target_vectors = [0.8 0.43 -0.2; -0.8 0.43 -0.2; 0.0 -0.9 -0.2];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Plotting data stuff
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define the plot Sequence ---> boolean operator(e.g., true = 1 and false = 0)
plt_stuff = struct;
plt_stuff.plot_single = 0;
plt_stuff.plot_double = 0;
plt_stuff.plot_triple = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set the figure file name and figure path
% plt_stuff.file_str = 'exp_directed_tran_rot_';
% plt_stuff.build_path = '~/Documents/MastersThesisUAS/CDC_2019/CDC_master/CDC_Master/CDC_master/build/';
% plt_stuff.fig_path = '~/Documents/MastersThesisUAS/CDC_2019/CDC_master/CDC_Master/fig_tikz/';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Pick the font size and line width
plt_stuff.fsize = 14;%10
plt_stuff.leg_fsize = 10;%8
plt_stuff.lval = 1.2;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Unpacking Sequence
disp('Begin Unpacking Data')

% First agent unpack data sequence
try
    data(1).dataPath = [path test 'Agent1/' data(1).file '.csv'];
    data(1).A = readtable(data(1).dataPath);
    data(1).index = find(data(1).A.v1_flightSequence > 0);
    data(1).index_form = find(data(1).A.v1_flightSequence == 6);
catch
    disp('No vehicle 1 data file.')
    data(1).A = 0;
end

% Second agent unpack data sequence 
try
    data(2).dataPath = [path test 'Agent2/' data(2).file '.csv'];
    data(2).A = readtable(data(2).dataPath);
    data(2).index = find(data(2).A.v2_flightSequence > 0);
    data(2).index_form = find(data(2).A.v2_flightSequence == 6);
catch
    disp('No vehicle 2 data file.')
    data(2).A = 0;
end


% Third agent unpack data sequence 
try
    data(3).dataPath = [path test 'Agent3/' data(3).file '.csv'];
    data(3).A = readtable(data(3).dataPath);
    data(3).index = find(data(3).A.v3_flightSequence > 0);
    data(3).index_form = find(data(3).A.v3_flightSequence == 6);
catch
    disp('No vehicle 3 data file.')
    data(3).A = 0;
end


%% Data Processor
disp('Processing Data')


% [agent,leader_agent,inter_agent] = AnalyzeThreeAgent(data(1),data(2),data(3),target_vectors);
[agent,leader_agent,inter_agent] = AnalyzeThreeAgentOutdoor(data,target_vectors);

% Define a nonlinear function for the outerloop formation control
% gain_analysis(data,agent,leader_agent,inter_agent);

% Compute the leader-agent-averaged errors and standard deviations
leader_agent_stats = computeLeaderAgentAverageErrors(agent,data,leader_agent,inter_agent);




%% Plotting Seqeunce
disp('Initiate Plotting Sequence')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Pick the index for which you want to plot
% plt_stuff.plot_index = agent(1).index;
plt_stuff.plot_index = agent(1).index_form;
% plt_stuff.plot_index = agent(1).index_virt;


if(plt_stuff.plot_single)
   disp('Single agent plots.')
   plotPositionAndVelocity(agent,plt_stuff)
%    plotPositionAndVelocity_R2T(agent,plt_stuff)
   
end

if(plt_stuff.plot_double)
   disp('Double agent plots.')
   plotPositionAndVelocity_TwoAgent(agent,plt_stuff)
   
   
end

if(plt_stuff.plot_triple)
    disp('Triple agent plots.')
    plotPositionAndVelocity_ThreeAgent(agent,plt_stuff)
end


disp('End main.')

%% Extra Plotting sequence
myPlots(agent,plt_stuff,data)




function myPlots(agent,plt_stuff,data)
    startTime = agent(1).time(plt_stuff.plot_index(1));
    endTime = 100; %agent(1).time(plt_stuff.plot_index(end));
    
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    figure
    subplot(3,1,1)
    plot(agent(1).time(plt_stuff.plot_index),agent(1).ctrl(plt_stuff.plot_index,1))
    xlim([startTime endTime])
    grid on
    
    subplot(3,1,2)
    plot(agent(1).time(plt_stuff.plot_index),agent(1).ctrl(plt_stuff.plot_index,2))
    xlim([startTime endTime])
    grid on
    
    subplot(3,1,3)
    plot(agent(1).time(plt_stuff.plot_index),agent(1).ctrl(plt_stuff.plot_index,3))
    xlim([startTime endTime])
    grid on
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Plot the relative positions of each agent
    
    
    target_vectors = [0.8 0.43 -0.2; -0.8 0.43 -0.2; 0.0 -0.9 -0.2];
%     dx_pi = cellfun(@str2num,data(1).A.v2_dx(plt_stuff.plot_index),'un',0);
    for i = 1:length(data(1).A.RelTime)
        qij(i,:) = lla2flat( [agent(1).pos_1(i,:)], agent(1).pos_2(i,1:2), 0, 0 ) - (target_vectors(1,1:3) - target_vectors(2,1:3));
        rel_pos(:,i) = getRelPos(agent(1).pos_2(i,1:2),agent(1).pos_1(i,1:2)) - (target_vectors(1,1:2) - target_vectors(2,1:2));
    end
    
    
%     plot(agent(1).time(plt_stuff.plot_index),rel_pos(:,plt_stuff.plot_index))
    
    
    

    dz = data(1).A.v1_zPos - data(1).A.v2_zPos;
    qij(:,3) = dz;
    
    
    figure
    subplot(3,1,1)
    plot(agent(1).time(plt_stuff.plot_index),qij(plt_stuff.plot_index,1),'r','linewidth',plt_stuff.lval)
    hold on 
    plot(agent(1).time(plt_stuff.plot_index),rel_pos(1,plt_stuff.plot_index),'b','linewidth',plt_stuff.lval)
    hold off
    xlim([startTime endTime])
    xlabel('$t$~(s)','interpreter','latex','FontSize',plt_stuff.fsize)
    ylabel('$e_1^{\rm T} \xi_{ij}$~(m)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    
    subplot(3,1,2)
    plot(agent(1).time(plt_stuff.plot_index),qij(plt_stuff.plot_index,2),'r','linewidth',plt_stuff.lval)
    hold on 
    plot(agent(1).time(plt_stuff.plot_index),rel_pos(2,plt_stuff.plot_index),'b','linewidth',plt_stuff.lval)
    hold off
    xlim([startTime endTime])
    xlabel('$t$~(s)','interpreter','latex','FontSize',plt_stuff.fsize)
    ylabel('$e_2^{\rm T} \xi_{ij}$~(m)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    
    subplot(3,1,3)
    plot(agent(1).time(plt_stuff.plot_index),qij(plt_stuff.plot_index,3),'b','linewidth',plt_stuff.lval)
    xlim([startTime endTime])
    xlabel('$t$~(s)','interpreter','latex','FontSize',plt_stuff.fsize)
    ylabel('$e_3^{\rm T} \xi_{ij}$~(m)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Plot the relative velocities of each agent
    
    pij = agent(1).vel_1 - agent(1).vel_2;
    
    figure
    subplot(3,1,1)
    plot(agent(1).time(plt_stuff.plot_index),pij(plt_stuff.plot_index,1),'b','linewidth',plt_stuff.lval)
    xlim([startTime endTime])
    xlabel('$t$~(s)','interpreter','latex','FontSize',plt_stuff.fsize)
    ylabel('$e_1^{\rm T} \rho_{ij}$~(m)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    
    subplot(3,1,2)
    plot(agent(1).time(plt_stuff.plot_index),pij(plt_stuff.plot_index,2),'b','linewidth',plt_stuff.lval)
    xlim([startTime endTime])
    xlabel('$t$~(s)','interpreter','latex','FontSize',plt_stuff.fsize)
    ylabel('$e_2^{\rm T} \rho_{ij}$~(m)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    
    subplot(3,1,3)
    plot(agent(1).time(plt_stuff.plot_index),pij(plt_stuff.plot_index,3),'b','linewidth',plt_stuff.lval)
    xlim([startTime endTime])
    xlabel('$t$~(s)','interpreter','latex','FontSize',plt_stuff.fsize)
    ylabel('$e_3^{\rm T} \rho_{ij}$~(m)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on

    

    
    
    
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     % Plot the positions of two agents
%     
%     
%     figure
%     subplot(3,1,1)
%     plot(agent(1).time(plt_stuff.plot_index),agent(1).pos_1(plt_stuff.plot_index,1),'b','linewidth',plt_stuff.lval)
%     hold on
%     plot(agent(1).time(plt_stuff.plot_index),agent(1).pos_2(plt_stuff.plot_index,1),'r','linewidth',plt_stuff.lval)
%     hold off
%     xlim([startTime endTime])
%     xlabel('$t$~(s)','interpreter','latex','FontSize',plt_stuff.fsize)
%     ylabel('Lat (deg)','interpreter','latex','FontSize',plt_stuff.fsize)
%     grid on
% 
%     subplot(3,1,2)
%     plot(agent(1).time(plt_stuff.plot_index),agent(1).pos_1(plt_stuff.plot_index,2),'b','linewidth',plt_stuff.lval)
%     hold on
%     plot(agent(1).time(plt_stuff.plot_index),agent(1).pos_2(plt_stuff.plot_index,2),'r','linewidth',plt_stuff.lval)
%     hold off
%     xlim([startTime endTime])
%     xlabel('$t$~(s)','interpreter','latex','FontSize',plt_stuff.fsize)
%     ylabel('Lon (deg)','interpreter','latex','FontSize',plt_stuff.fsize)
%     grid on
%     
%     subplot(3,1,3)
%     plot(agent(1).time(plt_stuff.plot_index),agent(1).pos_1(plt_stuff.plot_index,3),'b','linewidth',plt_stuff.lval)
%     hold on
%     plot(agent(1).time(plt_stuff.plot_index),agent(1).pos_2(plt_stuff.plot_index,3),'r','linewidth',plt_stuff.lval)
%     hold off
%     xlim([startTime endTime])
%     xlabel('$t$~(s)','interpreter','latex','FontSize',plt_stuff.fsize)
%     ylabel('Alt (m)','interpreter','latex','FontSize',plt_stuff.fsize)
%     grid on
%     
%     
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     % Plot the velocities of two agents
%     
%     
%     figure
%     subplot(3,1,1)
%     plot(agent(1).time(plt_stuff.plot_index),agent(1).vel_1(plt_stuff.plot_index,1),'b','linewidth',plt_stuff.lval)
%     hold on
%     plot(agent(1).time(plt_stuff.plot_index),agent(1).vel_2(plt_stuff.plot_index,1),'r','linewidth',plt_stuff.lval)
%     hold off
%     xlim([startTime endTime])
%     xlabel('$t$~(s)','interpreter','latex','FontSize',plt_stuff.fsize)
%     ylabel('$e_1^{\rm T} p_i ~(\frac{m}{s})$','interpreter','latex','FontSize',plt_stuff.fsize)
%     grid on
% 
%     subplot(3,1,2)
%     plot(agent(1).time(plt_stuff.plot_index),agent(1).vel_1(plt_stuff.plot_index,2),'b','linewidth',plt_stuff.lval)
%     hold on
%     plot(agent(1).time(plt_stuff.plot_index),agent(1).vel_2(plt_stuff.plot_index,2),'r','linewidth',plt_stuff.lval)
%     hold off
%     xlim([startTime endTime])
%     xlabel('$t$~(s)','interpreter','latex','FontSize',plt_stuff.fsize)
%     ylabel('$e_2^{\rm T} p_i ~(\frac{m}{s})$','interpreter','latex','FontSize',plt_stuff.fsize)
%     grid on
%     
%     subplot(3,1,3)
%     plot(agent(1).time(plt_stuff.plot_index),agent(1).vel_1(plt_stuff.plot_index,3),'b','linewidth',plt_stuff.lval)
%     hold on
%     plot(agent(1).time(plt_stuff.plot_index),agent(1).vel_2(plt_stuff.plot_index,3),'r','linewidth',plt_stuff.lval)
%     hold off
%     xlim([startTime endTime])
%     xlabel('$t$~(s)','interpreter','latex','FontSize',plt_stuff.fsize)
%     ylabel('$e_3^{\rm T} p_i ~(\frac{m}{s})$','interpreter','latex','FontSize',plt_stuff.fsize)
%     grid on
    
    
    

    
    
    

    end

    %% Functions used in data processing
    
    function output = getRelPos(pos1,pos2)
        c = 40074784;
        dy = (pos2(1,2) - pos1(1,2))*c*cos(deg2rad((pos1(1,1)+pos2(1,1))/2))/360;
        dx = (pos2(1,1) - pos1(1,1))*c/360;
        
        output = [dx,dy];
        
        
    end




