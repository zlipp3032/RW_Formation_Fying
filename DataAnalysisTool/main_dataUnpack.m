

clear
clc
close all


disp('Main data analysis tool for 3DR-SOLO formation flight tests.')

% Unpacking data stuff
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Prescribe path to the data files
path = '/home/zsl/Desktop/Data/';
test = '';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Pick the file strings
data = struct;
data(1).file = '2019_09_07__12_51_08_log_v1';
data(2).file = '2019_09_07__03_39_41_log_v2';
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
plt_stuff.plot_single = 1;
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
% plt_stuff.plot_index = agent(1).index_form;
plt_stuff.plot_index = agent(1).index_virt;


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
    endTime = agent(1).time(plt_stuff.plot_index(end));

    figure
    subplot(3,1,1)
    plot(agent(1).time(plt_stuff.plot_index),agent(1).vel_1(plt_stuff.plot_index,1),'b','linewidth',plt_stuff.lval)
    hold on
    plot(agent(1).time(plt_stuff.plot_index),agent(1).R2T.R2T_dot_1(1,plt_stuff.plot_index),'k --','linewidth',plt_stuff.lval)
    plot(agent(1).time(plt_stuff.plot_index),data(1).A.v1_vx_des(plt_stuff.plot_index),'r -.','linewidth',plt_stuff.lval)
    hold off
    ylabel('$e_{1}^{\rm T} p_i~(\frac{\rm m}{\rm s})$','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    xlim([startTime, endTime])
    leg_dummy = legend(); % sets the legend entries to nothing
    set(leg_dummy,'visible','off') % removes the legend from the plot
    set(gca,'xticklabel',[]) % gets rid of the labels on the x-axis

    subplot(3,1,2)
    plot(agent(1).time(plt_stuff.plot_index),agent(1).vel_1(plt_stuff.plot_index,2),'b','linewidth',plt_stuff.lval)
    hold on
    plot(agent(1).time(plt_stuff.plot_index),agent(1).R2T.R2T_dot_1(2,plt_stuff.plot_index),'k --','linewidth',plt_stuff.lval)
    plot(agent(1).time(plt_stuff.plot_index),data(1).A.v1_vy_des(plt_stuff.plot_index),'r -.','linewidth',plt_stuff.lval)
    hold off
    ylabel('$e_{2}^{\rm T} p_i~(\frac{\rm m}{\rm s})$','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    xlim([startTime, endTime])
    leg_dummy = legend(); % sets the legend entries to nothing
    set(leg_dummy,'visible','off') % removes the legend from the plot
    set(gca,'xticklabel',[]) % gets rid of the labels on the x-axis
    
    subplot(3,1,3)
    plot(agent(1).time(plt_stuff.plot_index),agent(1).vel_1(plt_stuff.plot_index,3),'b','linewidth',plt_stuff.lval)
    hold on
    plot(agent(1).time(plt_stuff.plot_index),agent(1).R2T.R2T_dot_1(3,plt_stuff.plot_index),'k --','linewidth',plt_stuff.lval)
    plot(agent(1).time(plt_stuff.plot_index),data(1).A.v1_vz_des(plt_stuff.plot_index),'r -.','linewidth',plt_stuff.lval)
    hold off
    ylabel('$e_{3}^{\rm T} p_i~(\frac{\rm m}{\rm s})$','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    xlim([startTime, endTime])
    leg_dummy = legend(); % sets the legend entries to nothing
    set(leg_dummy,'visible','off') % removes the legend from the plot
    set(gca,'xticklabel',[]) % gets rid of the labels on the x-axis
    
    
    

    figure
    subplot(3,1,1)
    plot(agent(1).time(plt_stuff.plot_index),agent(1).ctrl(plt_stuff.plot_index,1),'b','linewidth',plt_stuff.lval)
    grid on
    ylabel('$e_{1}^{\rm T} u_i~( \frac{\rm m}{\rm s^2} )$','interpreter','latex','FontSize',plt_stuff.fsize)
    xlim([startTime, endTime])
    
    subplot(3,1,2)
    plot(agent(1).time(plt_stuff.plot_index),agent(1).ctrl(plt_stuff.plot_index,2),'b','linewidth',plt_stuff.lval)
    grid on
    ylabel('$e_{1}^{\rm T} u_i~( \frac{\rm m}{\rm s^2} )$','interpreter','latex','FontSize',plt_stuff.fsize)
    xlim([startTime, endTime])
    
    subplot(3,1,3)
    plot(agent(1).time(plt_stuff.plot_index),agent(1).ctrl(plt_stuff.plot_index,3),'b','linewidth',plt_stuff.lval)
    grid on
    ylabel('$e_{3}^{\rm T} u_i~( \frac{\rm m}{\rm s^2} )$','interpreter','latex','FontSize',plt_stuff.fsize)
    xlim([startTime, endTime])
%     output = true;




end





