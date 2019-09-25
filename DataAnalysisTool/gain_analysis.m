function gain_analysis(data,agent,leader_agent,inter_agent)


%% Pick the formation control gains and control function


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% Pick the gains %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
alpha = 0.15;
beta = 0.86;
gamma = 0.3;
eta = 1.2;

alpha_z = 0.0;
beta_z = 0.0;
gamma_z = 0.3;
eta_z = 1.3;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% Pick the gains %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ku = 1.25;
kv = 1.25;
kw = 1.5;
ki = 0.5;
a = 7.7;
b = 1500;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Pick the Contr. Func. %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

nu = 0.25; f = @(x) x./sqrt(1 + nu*x.^2); 
% f = @(x) x;
% nu = 1; f = @(x) nu*tanh(x/nu);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Data processor

% Compute the PID control

%%%%%%%%%%%% On the list of things to do %%%%%%%%%%%%


% Compute the formation control
ALPHA = [alpha alpha alpha_z];
BETA = [beta beta beta_z];
GAMMA = [gamma gamma gamma_z];
ETA = [eta eta eta_z];






inter_pos_12_term = - ALPHA.*f(inter_agent(1).R2T_pos_error_12);
inter_vel_12_term = - BETA.*f(inter_agent(1).R2T_vel_error_12); 

inter_vel_13_term = - BETA.*f(inter_agent(1).R2T_vel_error_13);
inter_pos_13_term = - ALPHA.*f(inter_agent(1).R2T_pos_error_13);

leader_pos_1_term = - GAMMA.*f(leader_agent(1).R2T_pos_error_1);
leader_vel_1_term = - ETA.*f(leader_agent(1).R2T_vel_error_1);

inter_pos_1 = inter_pos_12_term + inter_pos_13_term;
inter_vel_1 = inter_vel_12_term + inter_vel_13_term;
leader_1 = leader_pos_1_term + leader_vel_1_term;



ug = [data(1).A.v1_lead_ugx(data(1).index), data(1).A.v1_lead_ugy(data(1).index), data(1).A.v1_lead_ugz(data(1).index)];
R_ddot = agent(1).R2T.R2T_ddot_1';
u_after = ug + R_ddot + inter_pos_1 + inter_vel_1 + leader_1;
u_error = u_after - [data(1).A.v1_ux(data(1).index), data(1).A.v1_uy(data(1).index), data(1).A.v1_uz(data(1).index)];


% Plot some things
figure
subplot(3,1,1)
plot(agent(1).time,u_after(:,1),'b')
hold on
plot(agent(1).time,data(1).A.v1_ux(data(1).index),'r--')
hold off

subplot(3,1,2)
plot(agent(1).time,u_after(:,2),'b')
hold on
plot(agent(1).time,data(1).A.v1_uy(data(1).index),'r--')
hold off

subplot(3,1,3)
plot(agent(1).time,u_after(:,3),'b')
hold on
plot(agent(1).time,data(1).A.v1_uz(data(1).index),'r--')
hold off


figure
plot(agent(1).time(data(1).index_form),u_error(data(1).index_form,:))

%% Analyze the middle-loop velocity controller
% gravity = 9.81;
% mass = 1.67;
% 
% 
% 
% % Compute the terms relating to the velocity stuff
% u_stuff = A1.v1_ux - ku.*(A1.v1_xVel - A1.v1_vx_des);
% v_stuff = A1.v1_uy - kv.*(A1.v1_yVel - A1.v1_vy_des);
% w_stuff = gravity - A1.v1_uz + kw.*(A1.v1_zVel - A1.v1_vz_des) + ki.*A1.v1_intZVelError;
% 
% % Compute the desired thrust and pitch
% thrust_after = mass.*w_stuff./(cos(A1.v1_roll).*cos(A1.v1_pitch));
% 
% temp_pitch = u_stuff.*cos(A1.v1_yaw) + v_stuff.*sin(A1.v1_yaw);
% pitch_after = atan(temp_pitch./(-w_stuff));
% 
% temp_roll = u_stuff.*sin(A1.v1_yaw).*cos(A1.v1_pitch_des) - v_stuff.*cos(A1.v1_yaw).*cos(A1.v1_pitch_des);
% roll_after = atan(temp_roll./(-w_stuff));
% 
% Thrust_est = mass.*(gravity + a_filthy)./(cos(A1.v1_roll).*cos(A1.v1_pitch));
% 
% figure
% subplot(3,1,1)
% plot(A1.RelTime(index_form1),thrust_after(index_form1),'b')
% hold on
% plot(A1.RelTime(index_form1),A1.v1_thrust_des(index_form1),'r --')
% hold off
% xlim([A1.RelTime(index_form1(1)) A1.RelTime(index_form1(end))])
% grid on
% 
% subplot(3,1,2)
% plot(A1.RelTime(index_form1),pitch_after(index_form1),'b')
% hold on
% plot(A1.RelTime(index_form1),A1.v1_pitch_des(index_form1),'r --')
% plot(A1.RelTime(index_form1),A1.v1_lin_pitch_des(index_form1),'m --')
% plot(A1.RelTime(index_form1),A1.v1_pitch(index_form1),'k -.')
% hold off
% xlim([A1.RelTime(index_form1(1)) A1.RelTime(index_form1(end))])
% grid on
% 
% subplot(3,1,3)
% plot(A1.RelTime(index_form1),roll_after(index_form1),'b')
% hold on
% plot(A1.RelTime(index_form1),A1.v1_roll_des(index_form1),'r --')
% plot(A1.RelTime(index_form1),A1.v1_lin_roll_des(index_form1),'m --')
% plot(A1.RelTime(index_form1),A1.v1_roll(index_form1),'k -.')
% hold off
% xlim([A1.RelTime(index_form1(1)) A1.RelTime(index_form1(end))])
% grid on
% 
% 
% w_stuff = gravity - A1.v1_uz + kw.*(A1.v1_zVel - A1.v1_vz_des) + ki.*A1.v1_intZVelError;
% quad_weight = [gravity*mass gravity*mass];
% 
% gravity_bit = [gravity gravity];
% uz_bit = - A1.v1_uz;
% vel_bit =  kw.*(A1.v1_zVel - A1.v1_vz_des);
% int_bit = ki.*A1.v1_intZVelError;
% 
% figure
% plot(A1.RelTime(index_form1),uz_bit(index_form1),'b')
% hold on
% plot(A1.RelTime(index_form1),vel_bit(index_form1),'r --')
% plot(A1.RelTime(index_form1),int_bit(index_form1),'m --')
% plot([A1.RelTime(index_form1(1)) A1.RelTime(index_form1(end))],gravity_bit,'k -.')
% hold off
% xlim([A1.RelTime(index_form1(1)) A1.RelTime(index_form1(end))])
% title('Thrust components')
% grid on
% 
% figure
% subplot(2,1,1)
% plot(A1.RelTime(index_form1),thrust_after(index_form1),'b')
% hold on
% plot(A1.RelTime(index_form1),A1.v1_thrust_des(index_form1),'r --')
% plot(A1.RelTime(index_form1),Thrust_est(index_form1),'m-')
% plot([A1.RelTime(index_form1(1)) A1.RelTime(index_form1(end))],quad_weight,'k-.')
% hold off
% xlim([A1.RelTime(index_form1(1)) A1.RelTime(index_form1(end))])
% title('Thrust')
% grid on
% 
% subplot(2,1,2)
% plot(A1.RelTime(index_form1),A1.v1_tPWM_des(index_form1),'b')
% xlim([A1.RelTime(index_form1(1)) A1.RelTime(index_form1(end))])
% title('Throttle PWM Output')
% grid on
















