function plotPositionAndVelocity(agent,plt_stuff)

disp('Plotting Position and Velocity')

startTime = agent(1).time(plt_stuff.plot_index(1));
endTime = agent(1).time(plt_stuff.plot_index(end));


% Plot the agent's positions
fig1 = figure;
subplot(3,1,1)
plot(agent(1).time(plt_stuff.plot_index),agent(1).pos_1(plt_stuff.plot_index,1),'b','linewidth',plt_stuff.lval)
hold on
plot(agent(1).time(plt_stuff.plot_index),agent(1).leader_pos(plt_stuff.plot_index,1),'k --','linewidth',plt_stuff.lval)
hold off
ylabel('$e_{1}^{\rm T} q_i$~(m)','interpreter','latex','FontSize',plt_stuff.fsize)
grid on
xlim([startTime, endTime])
leg_dummy = legend(); % sets the legend entries to nothing
set(leg_dummy,'visible','off') % removes the legend from the plot
set(gca,'xticklabel',[]) % gets rid of the labels on the x-axis

subplot(3,1,2)
plot(agent(1).time(plt_stuff.plot_index),agent(1).pos_1(plt_stuff.plot_index,2),'b','linewidth',plt_stuff.lval)
hold on
plot(agent(1).time(plt_stuff.plot_index),agent(1).leader_pos(plt_stuff.plot_index,2),'k --','linewidth',plt_stuff.lval)
hold off
ylabel('$e_{2}^{\rm T} q_i$~(m)','interpreter','latex','FontSize',plt_stuff.fsize)
grid on
xlim([startTime, endTime])
leg_dummy = legend(); % sets the legend entries to nothing
set(leg_dummy,'visible','off') % removes the legend from the plot
set(gca,'xticklabel',[]) % gets rid of the labels on the x-axis

subplot(3,1,3)
plot(agent(1).time(plt_stuff.plot_index),agent(1).pos_1(plt_stuff.plot_index,3),'b','linewidth',plt_stuff.lval)
hold on
plot(agent(1).time(plt_stuff.plot_index),agent(1).leader_pos(plt_stuff.plot_index,3),'k --','linewidth',plt_stuff.lval)
hold off
xlabel('$t$~(s)','interpreter','latex','FontSize',plt_stuff.fsize)
ylabel('$e_{3}^{\rm T} q_i$~(m)','interpreter','latex','FontSize',plt_stuff.fsize)
leg_pos = legend({'$q_1$','$q_2$','$q_3$','$q_g + R d_i$'},'Orientation','horizontal');%,'$q_2$','$q_g + R^{\rm T}d_2$','$q_3$','$q_g + R^{\rm T}d_3$');
legend boxoff
set(leg_pos,'interpreter','latex','Location','NorthEast','FontSize',plt_stuff.leg_fsize)
grid on
xlim([startTime, endTime])
% % Set figure properties and save it as tikz and pdf files.
% pdf_path_1 = [build_path file_str '_position.pdf'];
% tikz_path_1 = [fig_path file_str '_position.tikz'];
% % saveas(fig1,pdf_path_1);
% cleanfigure('handle',fig1)
% set(fig1,'Resize','on')
% matlab2tikz(tikz_path_1,'height', '\fheight', 'width', '\fwidth' );





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% Plot the agent's velocities
fig2 = figure;
subplot(3,1,1)
plot(agent(1).time(plt_stuff.plot_index),agent(1).vel_1(plt_stuff.plot_index,1),'b','linewidth',plt_stuff.lval)
hold on
plot(agent(1).time(plt_stuff.plot_index),agent(1).R2T.R2T_dot_1(1,plt_stuff.plot_index),'k --','linewidth',plt_stuff.lval)
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
hold off
xlabel('$t$~(s)','interpreter','latex','FontSize',plt_stuff.fsize)
ylabel('$e_{3}^{\rm T} p_i~(\frac{\rm m}{\rm s})$','interpreter','latex','FontSize',plt_stuff.fsize)
leg_pos = legend({'$p_1$','$p_2$','$p_3$','$p_g + R \Omega d_i$'},'Orientation','horizontal');%,'$q_2$','$q_g + R^{\rm T}d_2$','$q_3$','$q_g + R^{\rm T}d_3$');
legend boxoff
set(leg_pos,'interpreter','latex','Location','NorthEast','FontSize',plt_stuff.leg_fsize)
grid on
xlim([startTime, endTime])
% % Set figure properties and save it as tikz and pdf files.
% % pdf_path_1 = [build_path file_str '_velocity.pdf'];
% tikz_path_1 = [fig_path file_str '_velocity.tikz'];
% % saveas(fig1,pdf_path_1);
% cleanfigure('handle',fig1)
% set(fig1,'Resize','on')
% matlab2tikz(tikz_path_1,'height', '\fheight', 'width', '\fwidth' );



end