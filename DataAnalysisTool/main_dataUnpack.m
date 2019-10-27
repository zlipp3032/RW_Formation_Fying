

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
data(1).file = '2019_09_07__03_35_33_log_v1';
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


[agent,leader_agent,inter_agent] = AnalyzeThreeAgent(data(1),data(2),data(3),target_vectors);


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
%    plotPositionAndVelocity(agent,plt_stuff)
   plotPositionAndVelocity_R2T(agent,plt_stuff)
   
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






% plotSingleAgent_v1(A1,index_v1), plotSingleAgent_v2(A2,index_v2), plotSingleAgent_v3(A3,index_v3)
% plotTwoAgent(A1,index_v1,agent1_R2T,agent2_R2T,agent1_R2T_dot,agent2_R2T_dot,dist_error1,dist_error2)
% plotThreeAgent(A1,index1,agent1_R2T,agent2_R2T,agent3_R2T,agent1_R2T_dot,agent2_R2T_dot,agent3_R2T_dot,dist_error1,dist_error2,file_str,fig_path,build_path,ylmt,index_form1)
