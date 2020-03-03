

clear
clc
close all


disp('Main data analysis tool for 3DR-SOLO formation flight tests.')

% Unpacking data stuff
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Prescribe path to the data files
% path = '/Volumes/ZACK-DRIVE/FlightLogs/FormationTests/';
path =     '/Users/zlipp3032/Documents/MastersThesisUAS/Experiments/Data/Outdoor/FormationTests/';

test = 'Outdoor_022120/';


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Pick the file strings
data = struct;
data(1).file = '2020_01_21__19_10_46_log_v3';
data(2).file = '2019_09_07__12_54_44_log_v2';
data(3).file = '2020_01_21__11_17_10_log_v3';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Processing Data stuff
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the desired distance in the leader frame
% target_vectors = [0.8 0.43 -0.2; -0.8 0.43 -0.2; 0.0 -0.9 -0.2];
target_vectors = [3.0 2.0 0.0; -3.0 2.0 -3.0; 0.0 -3.0 -1.5];
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

plt_stuff.plot_sequence = 1; %0: Whole flight; 1: Virtual Leader; 2: Formation Control

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set the figure file name and figure path
% plt_stuff.file_str = 'exp_directed_tran_rot_';
% plt_stuff.build_path = '~/Documents/MastersThesisUAS/CDC_2019/CDC_master/CDC_Master/CDC_master/build/';
% plt_stuff.fig_path = '~/Documents/MastersThesisUAS/CDC_2019/CDC_master/CDC_Master/fig_tikz/';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Pick the font size and line width
plt_stuff.fsize = 16;%10
plt_stuff.leg_fsize = 12;%8
plt_stuff.lval = 1.2;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Unpacking Sequence
disp('Begin Unpacking Data')

% First agent unpack data sequence
try
%     data(1).dataPath = [path test 'Agent1/' data(1).file '.csv'];
    data(1).dataPath = [path test 'Agent1/' data(1).file '.csv'];
    data(1).A = readtable(data(1).dataPath);
    data(1).index = find(data(1).A.v3_flightSequence > 0);
    data(1).index_form = find(data(1).A.v3_flightSequence == 6);
    data(1).index_virt = find(data(1).A.v3_flightSequence == 5);
    data(1).index_rtl = find(data(1).A.v3_flightSequence == 4);
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
    data(2).index_virt = find(data(2).A.v2_flightSequence == 5);
    data(2).index_rtl = find(data(2).A.v2_flightSequence == 4);
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
    data(3).index_virt = find(data(3).A.v3_flightSequence == 5);
    data(3).index_rtl = find(data(3).A.v3_flightSequence == 4);
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

% switch plt_stuff.plot_sequence
%     case 0
%         plt_stuff.plot_index = agent(1).index;
%     case 1
%         plt_stuff.plot_index = agent(1).index_virt;
%     case 2
%         plt_stuff.plot_index = agent(1).index_form;
%     otherwise
%         disp('Pick new plotting sequence.')
%         disp('0: Whole flight')
%         disp('1: Virtual Leader')
%         disp('2: Formation Control')
% end

      

if(plt_stuff.plot_single)
   disp('Single agent plots.')
   plotPositionAndVelocity(agent,plt_stuff,data)
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
% myPlots(agent,plt_stuff,data,target_vectors)

myPlots_mid_test(plt_stuff,data)

function myPlots_mid_test(plt_stuff,data)
    
    index = data(1).index_virt;
    
    startTime = data(1).A.RelTime(index(1));
    endTime = data(1).A.RelTime(index(end));
    
    
    for i = 1:length(index)
        agent_pos_1(i,:) = lla2flat([data(1).A.v3_lat(index(i)),data(1).A.v3_lon(index(i)),data(1).A.v3_zPos(index(i))],[data(1).A.v3_lead_lat(index(i)) data(1).A.v3_lead_lon(index(i))],0,0);
        
        lead_pos(i,:) = lla2flat([data(1).A.v3_lead_lat(index(i)) data(1).A.v3_lead_lon(index(i)),data(1).A.v3_leadZPos(index(i))],[data(1).A.v3_lead_lat(index(i)) data(1).A.v3_lead_lon(index(i))],0,0);
        lead_pos(i,:) = lead_pos(i,:) + [data(1).A.v3_leadXPos(index(i)),data(1).A.v3_leadYPos(index(i)),0];
       
    end

    
    
    
    
    
    figure
    subplot(3,1,1)
    plot(data(1).A.RelTime(index),agent_pos_1(:,1),'b','linewidth',plt_stuff.lval)
    hold on
    plot(data(1).A.RelTime(index),lead_pos(:,1),'k --','linewidth',plt_stuff.lval)
    hold off
    xlim([startTime endTime])
    ylabel('$e_{1}^{\rm T} q_i$~(m)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    xlim([startTime, endTime])
    leg_dummy = legend(); % sets the legend entries to nothing
    set(leg_dummy,'visible','off') % removes the legend from the plot
    set(gca,'xticklabel',[]) % gets rid of the labels on the x-axis
%     ylim([-1 0.1])
    
    subplot(3,1,2)
    plot(data(1).A.RelTime(index),agent_pos_1(:,2),'b','linewidth',plt_stuff.lval)
    hold on
    plot(data(1).A.RelTime(index),lead_pos(:,2),'k --','linewidth',plt_stuff.lval)
    hold off
    xlim([startTime endTime])
    ylabel('$e_{2}^{\rm T} q_i $~(m)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    xlim([startTime, endTime])
    leg_dummy = legend(); % sets the legend entries to nothing
    set(leg_dummy,'visible','off') % removes the legend from the plot
    set(gca,'xticklabel',[]) % gets rid of the labels on the x-axis
%     ylim([-0.1 1])
    
    subplot(3,1,3)
    plot(data(1).A.RelTime(index),-agent_pos_1(:,3),'b','linewidth',plt_stuff.lval)
    hold on
    plot(data(1).A.RelTime(index),-lead_pos(:,3),'k --','linewidth',plt_stuff.lval)
    hold off
    xlim([startTime endTime])
    ylabel('$e_{3}^{\rm T} q_i $~(m)','interpreter','latex','FontSize',plt_stuff.fsize)
    xlabel('$t$~(s)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    xlim([startTime, endTime])
    leggy = legend({'$q$','$q_{\rm g}$'},'orientation','horizontal'); % sets the legend entries to nothing
    legend boxoff
    set(leggy,'interpreter','latex','FontSize',plt_stuff.leg_fsize) % removes the legend from the plot
    
    
    
    
    figure
    subplot(3,1,1)
    plot(data(1).A.RelTime(index),data(1).A.v3_xVel(index),'b','linewidth',plt_stuff.lval)
    hold on
    plot(data(1).A.RelTime(index),data(1).A.v3_vx_des(index),'r','linewidth',plt_stuff.lval)
%     plot(data(1).A.RelTime(index),data(1).A.v3_vx_hat(index),'c-.','linewidth',plt_stuff.lval)
    plot(data(1).A.RelTime(index),data(1).A.v3_leadXVel(index),'k --','linewidth',plt_stuff.lval)
    hold off
    xlim([startTime endTime])
    ylabel('$e_{1}^{\rm T} p_i$~(m/s)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    xlim([startTime, endTime])
    leg_dummy = legend(); % sets the legend entries to nothing
    set(leg_dummy,'visible','off') % removes the legend from the plot
    set(gca,'xticklabel',[]) % gets rid of the labels on the x-axis
%     ylim([-0.5 0.5])
    
    subplot(3,1,2)
    plot(data(1).A.RelTime(index),data(1).A.v3_yVel(index),'b','linewidth',plt_stuff.lval)
    hold on
    plot(data(1).A.RelTime(index),data(1).A.v3_vy_des(index),'r','linewidth',plt_stuff.lval)
%     plot(data(1).A.RelTime(index),data(1).A.v3_vy_hat(index),'c-.','linewidth',plt_stuff.lval)
    plot(data(1).A.RelTime(index),data(1).A.v3_leadYVel(index),'k --','linewidth',plt_stuff.lval)
    hold off
    xlim([startTime endTime])
    ylabel('$e_{2}^{\rm T} p_i $~(m/s)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    xlim([startTime, endTime])
    leg_dummy = legend(); % sets the legend entries to nothing
    set(leg_dummy,'visible','off') % removes the legend from the plot
    set(gca,'xticklabel',[]) % gets rid of the labels on the x-axis
%     ylim([-0.5 0.5])
    
    subplot(3,1,3)
    plot(data(1).A.RelTime(index),data(1).A.v3_zVel(index),'b','linewidth',plt_stuff.lval)
    hold on
    plot(data(1).A.RelTime(index),data(1).A.v3_vz_des(index),'r','linewidth',plt_stuff.lval)
%     plot(data(1).A.RelTime(index),data(1).A.v1_vz_hat(index),'c-.','linewidth',plt_stuff.lval)
%     plot(0,0,'c-.')
    plot(data(1).A.RelTime(index),data(1).A.v3_leadZVel(index),'k --','linewidth',plt_stuff.lval)
    hold off
    ylabel('$e_{3}^{\rm T} p_i $~(m/s)','interpreter','latex','FontSize',plt_stuff.fsize)
    xlabel('$t$~(s)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    xlim([startTime, endTime])
    leggy = legend({'$p$','$v_d$','$\hat{v}_d$','$p_{\rm g}$'},'orientation','horizontal'); % sets the legend entries to nothing
    legend boxoff
    set(leggy,'interpreter','latex','FontSize',plt_stuff.leg_fsize) % removes the legend from the plot
    

    
    
    
    
    min_time = input('Time_min for cost evaluation: \n');
    max_time = input('Time_max for cost evaluation: \n');
        
    error =  agent_pos_1 - lead_pos;
    error_vel = [data(1).A.v3_xVel(index),data(1).A.v3_yVel(index),data(1).A.v3_zVel(index)] - [data(1).A.v3_leadXVel(index),data(1).A.v3_leadYVel(index),data(1).A.v3_leadZVel(index)];
    
    
    index_cost = find(data(1).A.RelTime(index) > min_time);
    tempdex_cost = find(data(1).A.RelTime(index(index_cost)) < max_time);
    
    
%     figure
%     plot(data(1).A.RelTime(index),agent_pos_1(:,1))
%     hold on
%     plot(data(1).A.RelTime(index(index_cost)),agent_pos_1(index_cost,1),'r --')
    
    
    
    Cost_pos = (1/2)*sum(error(index_cost(tempdex_cost),:).^2,1)/length(index_cost(tempdex_cost))
    Cost_vel = (1/2)*sum(error_vel(index_cost(tempdex_cost),:).^2,1)/length(index_cost(tempdex_cost))



end





































function myPlots(agent,plt_stuff,data,di)
    startTime = agent(1).time(plt_stuff.plot_index(1));
    endTime = agent(1).time(plt_stuff.plot_index(end));
    
    
    agent_vel_1 = agent(1).vel_1(plt_stuff.plot_index,:);
    agent_vel_2 = agent(1).vel_2(plt_stuff.plot_index,:);
    agent_vel_3 = agent(1).vel_3(plt_stuff.plot_index,:);
    
    lead_vel = agent(1).leader_vel(plt_stuff.plot_index,:);
    
    for i = 1:length(agent(1).time(plt_stuff.plot_index))
        agent_pos_1(i,:) = lla2flat([agent(1).pos_1(plt_stuff.plot_index(i),1),agent(1).pos_1(plt_stuff.plot_index(i),2),agent(1).pos_1(plt_stuff.plot_index(i),3)],[data(1).A.v1_lead_lat(plt_stuff.plot_index(i)) data(1).A.v1_lead_lon(plt_stuff.plot_index(i))],0,0);
        agent_pos_2(i,:) = lla2flat([agent(1).pos_2(plt_stuff.plot_index(i),1),agent(1).pos_2(plt_stuff.plot_index(i),2),agent(1).pos_2(plt_stuff.plot_index(i),3)],[data(1).A.v1_lead_lat(plt_stuff.plot_index(i)) data(1).A.v1_lead_lon(plt_stuff.plot_index(i))],0,0);
        agent_pos_3(i,:) = lla2flat([agent(1).pos_3(plt_stuff.plot_index(i),1),agent(1).pos_3(plt_stuff.plot_index(i),2),agent(1).pos_3(plt_stuff.plot_index(i),3)],[data(1).A.v1_lead_lat(plt_stuff.plot_index(i)) data(1).A.v1_lead_lon(plt_stuff.plot_index(i))],0,0);
        lead_pos(i,:) = lla2flat([agent(1).leader_pos(plt_stuff.plot_index(i),1),agent(1).leader_pos(plt_stuff.plot_index(i),2),agent(1).leader_pos(plt_stuff.plot_index(i),3)],[data(1).A.v1_lead_lat(plt_stuff.plot_index(i)) data(1).A.v1_lead_lon(plt_stuff.plot_index(i))],0,0);

        
        % Look at potential velocity controller
        kp = 0.8;
        vd(i,:) = kp.*([data(1).A.v1_vx_des(data(1).index_form(i)), data(1).A.v1_vy_des(data(1).index_form(i)), data(1).A.v1_vz_des(data(1).index_form(i))] - agent_vel_1(i,:));
        
    end
    
    
    
    figure
    subplot(3,1,1)
    plot(vd(:,1),'b')
    hold on
    plot(data(1).A.v1_ux(data(1).index_form),'r')
    plot(data(1).A.v1_vx_des(data(1).index_form),'c')
    plot(agent_vel_1(:,1),'g')
    hold off
    grid on
    
    subplot(3,1,2)
    plot(vd(:,2),'b')
    hold on
    plot(data(1).A.v1_uy(data(1).index_form),'r')
    plot(data(1).A.v1_vy_des(data(1).index_form),'c')
    plot(agent_vel_1(:,2),'g')
    hold off
    grid on
    
    subplot(3,1,3)
    plot(vd(:,3),'b')
    hold on
    plot(data(1).A.v1_uz(data(1).index_form),'r')
    plot(data(1).A.v1_vz_des(data(1).index_form),'c')
    plot(agent_vel_1(:,3),'g')
    hold off
    grid on
    
    
    
    
    
    ones_mat = ones(length(agent(1).time(plt_stuff.plot_index)),3);
    
    lead_R2T_1 = -lead_pos + di(1,:).*ones_mat;
    lead_R2T_2 = -lead_pos + di(2,:).*ones_mat;
    lead_R2T_3 = -lead_pos + di(3,:).*ones_mat;
    
    pos_12 = agent_pos_1 - agent_pos_2;
    pos_13 = agent_pos_1 - agent_pos_3;
    pos_23 = agent_pos_2 - agent_pos_3;
    td = [startTime endTime]';
    d12 = [(di(1,:) - di(2,:));(di(1,:) - di(2,:))];
    d13 = [(di(1,:) - di(3,:));(di(1,:) - di(3,:))];
    d23 = [(di(2,:) - di(3,:));(di(2,:) - di(3,:))];
    
    
    
    % Plot the inter-agent positions
    fig0 = figure;
    subplot(3,1,1)
    plot(agent(1).time(plt_stuff.plot_index),pos_12(:,1),'b','linewidth',plt_stuff.lval)
    hold on
    plot(agent(1).time(plt_stuff.plot_index),pos_13(:,1),'r','linewidth',plt_stuff.lval)
    plot(agent(1).time(plt_stuff.plot_index),pos_23(:,1),'g','linewidth',plt_stuff.lval)
    plot(td,d12(:,1),'b --','linewidth',plt_stuff.lval)
    plot(td,d13(:,1),'r --','linewidth',plt_stuff.lval)
    plot(td,d23(:,1),'g --','linewidth',plt_stuff.lval)
    hold off
    ylabel('$e_{1}^{\rm T} (q_i - q_j)$~(m)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    xlim([startTime, endTime])
    leg_dummy = legend(); % sets the legend entries to nothing
    set(leg_dummy,'visible','off') % removes the legend from the plot
    set(gca,'xticklabel',[]) % gets rid of the labels on the x-axis
    
    subplot(3,1,2)
    plot(agent(1).time(plt_stuff.plot_index),pos_12(:,2),'b','linewidth',plt_stuff.lval)
    hold on
    plot(agent(1).time(plt_stuff.plot_index),pos_13(:,2),'r','linewidth',plt_stuff.lval)
    plot(agent(1).time(plt_stuff.plot_index),pos_23(:,2),'g','linewidth',plt_stuff.lval)
    plot(td,d12(:,2),'b --','linewidth',plt_stuff.lval)
    plot(td,d13(:,2),'r --','linewidth',plt_stuff.lval)
    plot(td,d23(:,2),'g --','linewidth',plt_stuff.lval)
    hold off
    ylabel('$e_{2}^{\rm T} (q_i - q_j)$~(m)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    xlim([startTime, endTime])
    leg_dummy = legend(); % sets the legend entries to nothing
    set(leg_dummy,'visible','off') % removes the legend from the plot
    set(gca,'xticklabel',[]) % gets rid of the labels on the x-axis
    
    subplot(3,1,3)
    plot(agent(1).time(plt_stuff.plot_index),-pos_12(:,3),'b','linewidth',plt_stuff.lval)
    hold on
    plot(agent(1).time(plt_stuff.plot_index),-pos_13(:,3),'r','linewidth',plt_stuff.lval)
    plot(agent(1).time(plt_stuff.plot_index),-pos_23(:,3),'g','linewidth',plt_stuff.lval)
    plot(td,d12(:,3),'b --','linewidth',plt_stuff.lval)
    plot(td,d13(:,3),'r --','linewidth',plt_stuff.lval)
    plot(td,d23(:,3),'g --','linewidth',plt_stuff.lval)
    hold off
    ylabel('$e_{3}^{\rm T} (q_i - q_j)$~(m)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    xlim([startTime, endTime])
    leg_fig1 = legend({'$(q_1 - q_2)$','$(q_1 - q_3)$','$(q_2 - q_3)$','$\delta_i + \delta_j$'},'orientation','horizontal'); % sets the legend entries to nothing
    legend boxoff
    set(leg_fig1,'interpreter','latex','FontSize',plt_stuff.leg_fsize) % removes the legend from the plot
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    


    % Plot the agent's positions
    fig1 = figure;
    subplot(3,1,1)
    plot(agent(1).time(plt_stuff.plot_index),agent_pos_1(:,1),'b','linewidth',plt_stuff.lval)
    hold on
    plot(agent(1).time(plt_stuff.plot_index),agent_pos_2(:,1),'r','linewidth',plt_stuff.lval)
    plot(agent(1).time(plt_stuff.plot_index),agent_pos_3(:,1),'g','linewidth',plt_stuff.lval)
    plot(agent(1).time(plt_stuff.plot_index),lead_R2T_1(:,1),'b --','linewidth',plt_stuff.lval)
    plot(agent(1).time(plt_stuff.plot_index),lead_R2T_2(:,1),'r --','linewidth',plt_stuff.lval)
    plot(agent(1).time(plt_stuff.plot_index),lead_R2T_3(:,1),'g --','linewidth',plt_stuff.lval)
    hold off
    ylabel('$e_{1}^{\rm T} q_i$~(m)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    xlim([startTime, endTime])
    leg_dummy = legend(); % sets the legend entries to nothing
    set(leg_dummy,'visible','off') % removes the legend from the plot
    set(gca,'xticklabel',[]) % gets rid of the labels on the x-axis
    
    subplot(3,1,2)
    plot(agent(1).time(plt_stuff.plot_index),agent_pos_1(:,2),'b','linewidth',plt_stuff.lval)
    hold on
    plot(agent(1).time(plt_stuff.plot_index),agent_pos_2(:,2),'r','linewidth',plt_stuff.lval)
    plot(agent(1).time(plt_stuff.plot_index),agent_pos_3(:,2),'g','linewidth',plt_stuff.lval)
    plot(agent(1).time(plt_stuff.plot_index),lead_R2T_1(:,2),'b --','linewidth',plt_stuff.lval)
    plot(agent(1).time(plt_stuff.plot_index),lead_R2T_2(:,2),'r --','linewidth',plt_stuff.lval)
    plot(agent(1).time(plt_stuff.plot_index),lead_R2T_3(:,2),'g --','linewidth',plt_stuff.lval)
    hold off
    ylabel('$e_{2}^{\rm T} q_i$~(m)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    xlim([startTime, endTime])
    leg_dummy = legend(); % sets the legend entries to nothing
    set(leg_dummy,'visible','off') % removes the legend from the plot
    set(gca,'xticklabel',[]) % gets rid of the labels on the x-axis
    
    subplot(3,1,3)
    plot(agent(1).time(plt_stuff.plot_index),-agent_pos_1(:,3),'b','linewidth',plt_stuff.lval)
    hold on
    plot(agent(1).time(plt_stuff.plot_index),-agent_pos_2(:,3),'r','linewidth',plt_stuff.lval)
    plot(agent(1).time(plt_stuff.plot_index),-agent_pos_3(:,3),'g','linewidth',plt_stuff.lval)
    plot(agent(1).time(plt_stuff.plot_index),lead_R2T_1(:,3),'b --','linewidth',plt_stuff.lval)
    plot(agent(1).time(plt_stuff.plot_index),lead_R2T_2(:,3),'r --','linewidth',plt_stuff.lval)
    plot(agent(1).time(plt_stuff.plot_index),lead_R2T_3(:,3),'g --','linewidth',plt_stuff.lval)
    hold off
    ylabel('$e_{3}^{\rm T} q_i$~(m)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    xlim([startTime, endTime])
    leg_fig1 = legend({'$q_1$','$q_2$','$q_3$','$q_{\rm g} + \delta_i$'},'orientation','horizontal'); % sets the legend entries to nothing
    legend boxoff
    set(leg_fig1,'interpreter','latex','FontSize',plt_stuff.leg_fsize) % removes the legend from the plot
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
    
    
    % Plot the agent's velocities
    fig2 = figure;
    subplot(3,1,1)
    plot(agent(1).time(plt_stuff.plot_index),agent_vel_1(:,1),'b','linewidth',plt_stuff.lval)
    hold on
    plot(agent(1).time(plt_stuff.plot_index),agent_vel_2(:,1),'r','linewidth',plt_stuff.lval)
    plot(agent(1).time(plt_stuff.plot_index),agent_vel_3(:,1),'g','linewidth',plt_stuff.lval)
    plot(agent(1).time(plt_stuff.plot_index),lead_vel(:,1),'k --','linewidth',plt_stuff.lval)
%     plot(agent(1).time(plt_stuff.plot_index),lead_R2T_2(:,1),'r --','linewidth',plt_stuff.lval)
%     plot(agent(1).time(plt_stuff.plot_index),lead_R2T_3(:,1),'g --','linewidth',plt_stuff.lval)
    hold off
    ylabel('$e_{1}^{\rm T} p_i$~(m/s)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    xlim([startTime, endTime])
    leg_dummy = legend(); % sets the legend entries to nothing
    set(leg_dummy,'visible','off') % removes the legend from the plot
    set(gca,'xticklabel',[]) % gets rid of the labels on the x-axis
    
    subplot(3,1,2)
    plot(agent(1).time(plt_stuff.plot_index),agent_vel_1(:,2),'b','linewidth',plt_stuff.lval)
    hold on
    plot(agent(1).time(plt_stuff.plot_index),agent_vel_2(:,2),'r','linewidth',plt_stuff.lval)
    plot(agent(1).time(plt_stuff.plot_index),agent_vel_3(:,2),'g','linewidth',plt_stuff.lval)
    plot(agent(1).time(plt_stuff.plot_index),lead_vel(:,2),'k --','linewidth',plt_stuff.lval)
%     plot(agent(1).time(plt_stuff.plot_index),lead_R2T_2(:,2),'r --','linewidth',plt_stuff.lval)
%     plot(agent(1).time(plt_stuff.plot_index),lead_R2T_3(:,2),'g --','linewidth',plt_stuff.lval)
    hold off
    ylabel('$e_{2}^{\rm T} p_i$~(m/s)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    xlim([startTime, endTime])
    leg_dummy = legend(); % sets the legend entries to nothing
    set(leg_dummy,'visible','off') % removes the legend from the plot
    set(gca,'xticklabel',[]) % gets rid of the labels on the x-axis
    
    subplot(3,1,3)
    plot(agent(1).time(plt_stuff.plot_index),agent_vel_1(:,3),'b','linewidth',plt_stuff.lval)
    hold on
    plot(agent(1).time(plt_stuff.plot_index),agent_vel_2(:,3),'r','linewidth',plt_stuff.lval)
    plot(agent(1).time(plt_stuff.plot_index),agent_vel_3(:,3),'g','linewidth',plt_stuff.lval)
    plot(agent(1).time(plt_stuff.plot_index),lead_vel(:,3),'k --','linewidth',plt_stuff.lval)
%     plot(agent(1).time(plt_stuff.plot_index),lead_R2T_2(:,3),'r --','linewidth',plt_stuff.lval)
%     plot(agent(1).time(plt_stuff.plot_index),lead_R2T_3(:,3),'g --','linewidth',plt_stuff.lval)
    hold off
    ylabel('$e_{3}^{\rm T} p_i$~(m/s)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    xlim([startTime, endTime])
    leg_fig2 = legend({'$p_1$','$p_2$','$p_3$','$p_{\rm g} + \dot{\delta}_i$'},'orientation','horizontal'); % sets the legend entries to nothing
    legend boxoff
    set(leg_fig2,'interpreter','latex','FontSize',plt_stuff.leg_fsize) % removes the legend from the plot


    
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
    % Plot agent one's input
    fig3 = figure;
    subplot(3,1,1)
    plot(agent(1).time(plt_stuff.plot_index),data(1).A.v1_vx_des(data(1).index_form),'b','linewidth',plt_stuff.lval)
    hold on
    plot(agent(1).time(plt_stuff.plot_index),data(1).A.v1_xVel(data(1).index_form),'r -.','linewidth',plt_stuff.lval)
    hold off
    ylabel('$e_{1}^{\rm T} p_1$~(m/s)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    xlim([startTime, endTime])
    leg_dummy = legend(); % sets the legend entries to nothing
    set(leg_dummy,'visible','off') % removes the legend from the plot
    set(gca,'xticklabel',[]) % gets rid of the labels on the x-axis
    
    subplot(3,1,2)
    plot(agent(1).time(plt_stuff.plot_index),data(1).A.v1_vy_des(data(1).index_form),'b','linewidth',plt_stuff.lval)
    hold on
    plot(agent(1).time(plt_stuff.plot_index),data(1).A.v1_yVel(data(1).index_form),'r -.','linewidth',plt_stuff.lval)
    hold off
    ylabel('$e_{2}^{\rm T} p_1$~(m/s)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    xlim([startTime, endTime])
    leg_dummy = legend(); % sets the legend entries to nothing
    set(leg_dummy,'visible','off') % removes the legend from the plot
    set(gca,'xticklabel',[]) % gets rid of the labels on the x-axis
    
    subplot(3,1,3)
    plot(agent(1).time(plt_stuff.plot_index),data(1).A.v1_vz_des(data(1).index_form),'b','linewidth',plt_stuff.lval)
    hold on
	plot(agent(1).time(plt_stuff.plot_index),data(1).A.v1_zVel(data(1).index_form),'r -.','linewidth',plt_stuff.lval)
    hold off
    ylabel('$e_{3}^{\rm T} p_1$~(m/s)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    xlim([startTime, endTime])
    leg_fig3 = legend({'$v_{1,d}$','$p_{1}$'},'orientation','horizontal'); % sets the legend entries to nothing
    legend boxoff
    set(leg_fig3,'interpreter','latex','FontSize',plt_stuff.leg_fsize) % removes the legend from the plot
    
    
    
    
    
    
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
    % Plot agent two's input
    fig4 = figure;
    subplot(3,1,1)
    plot(data(2).A.RelTime(data(2).index_form),data(2).A.v2_vx_des(data(2).index_form),'b','linewidth',plt_stuff.lval)
    hold on
    plot(data(2).A.RelTime(data(2).index_form),data(2).A.v2_xVel(data(2).index_form),'r -.','linewidth',plt_stuff.lval)
    hold off
    ylabel('$e_{1}^{\rm T} p_2$~(m/s)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    xlim([data(2).A.RelTime(data(2).index_form(1)), data(2).A.RelTime(data(2).index_form(end))])
    leg_dummy = legend(); % sets the legend entries to nothing
    set(leg_dummy,'visible','off') % removes the legend from the plot
    set(gca,'xticklabel',[]) % gets rid of the labels on the x-axis
    
    subplot(3,1,2)
    plot(data(2).A.RelTime(data(2).index_form),data(2).A.v2_vy_des(data(2).index_form),'b','linewidth',plt_stuff.lval)
    hold on
    plot(data(2).A.RelTime(data(2).index_form),data(2).A.v2_yVel(data(2).index_form),'r -.','linewidth',plt_stuff.lval)
    hold off
    ylabel('$e_{2}^{\rm T} p_2$~(m/s)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    xlim([data(2).A.RelTime(data(2).index_form(1)), data(2).A.RelTime(data(2).index_form(end))])
    leg_dummy = legend(); % sets the legend entries to nothing
    set(leg_dummy,'visible','off') % removes the legend from the plot
    set(gca,'xticklabel',[]) % gets rid of the labels on the x-axis
    
    subplot(3,1,3)
    plot(data(2).A.RelTime(data(2).index_form),data(2).A.v2_vz_des(data(2).index_form),'b','linewidth',plt_stuff.lval)
    hold on
	plot(data(2).A.RelTime(data(2).index_form),data(2).A.v2_zVel(data(2).index_form),'r -.','linewidth',plt_stuff.lval)
    hold off
    ylabel('$e_{3}^{\rm T} p_2$~(m/s)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    xlim([data(2).A.RelTime(data(2).index_form(1)), data(2).A.RelTime(data(2).index_form(end))])
    leg_fig4 = legend({'$v_{2,d}$','$p_{2}$'},'orientation','horizontal'); % sets the legend entries to nothing
    legend boxoff
    set(leg_fig4,'interpreter','latex','FontSize',plt_stuff.leg_fsize) % removes the legend from the plot

    
    
    
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
    % Plot agent three's input
    fig5 = figure;
    subplot(3,1,1)
    plot(data(3).A.RelTime(data(3).index_form),data(3).A.v3_vx_des(data(3).index_form),'b','linewidth',plt_stuff.lval)
    hold on
    plot(data(3).A.RelTime(data(3).index_form),data(3).A.v3_xVel(data(3).index_form),'r -.','linewidth',plt_stuff.lval)
    hold off
    ylabel('$e_{1}^{\rm T} p_3$~(m/s)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    xlim([data(3).A.RelTime(data(3).index_form(1)), data(3).A.RelTime(data(3).index_form(end))])
    leg_dummy = legend(); % sets the legend entries to nothing
    set(leg_dummy,'visible','off') % removes the legend from the plot
    set(gca,'xticklabel',[]) % gets rid of the labels on the x-axis
    
    subplot(3,1,2)
    plot(data(3).A.RelTime(data(3).index_form),data(3).A.v3_vy_des(data(3).index_form),'b','linewidth',plt_stuff.lval)
    hold on
    plot(data(3).A.RelTime(data(3).index_form),data(3).A.v3_yVel(data(3).index_form),'r -.','linewidth',plt_stuff.lval)
    hold off
    ylabel('$e_{2}^{\rm T} p_3$~(m/s)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    xlim([data(3).A.RelTime(data(3).index_form(1)), data(3).A.RelTime(data(3).index_form(end))])
    leg_dummy = legend(); % sets the legend entries to nothing
    set(leg_dummy,'visible','off') % removes the legend from the plot
    set(gca,'xticklabel',[]) % gets rid of the labels on the x-axis
    
    subplot(3,1,3)
    plot(data(3).A.RelTime(data(3).index_form),data(3).A.v3_vz_des(data(3).index_form),'b','linewidth',plt_stuff.lval)
    hold on
	plot(data(3).A.RelTime(data(3).index_form),data(3).A.v3_zVel(data(3).index_form),'r -.','linewidth',plt_stuff.lval)
    hold off
    ylabel('$e_{3}^{\rm T} p_3$~(m/s)','interpreter','latex','FontSize',plt_stuff.fsize)
    grid on
    xlim([data(3).A.RelTime(data(3).index_form(1)), data(3).A.RelTime(data(3).index_form(end))])
    leg_fig5 = legend({'$v_{3,d}$','$p_{3}$'},'orientation','horizontal'); % sets the legend entries to nothing
    legend boxoff
    set(leg_fig5,'interpreter','latex','FontSize',plt_stuff.leg_fsize) % removes the legend from the plot
    
    
    
    
    
    

end

    %% Functions used in data processing
    
    function output = getRelPos(pos1,pos2)
        c = 40074784;
        dy = (pos2(1,2) - pos1(1,2))*c*cos(deg2rad((pos1(1,1)+pos2(1,1))/2))/360;
        dx = (pos2(1,1) - pos1(1,1))*c/360;
        
        output = [dx,dy];
        
        
    end



