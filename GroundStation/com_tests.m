



clc
clear
close all


Ts = 0.055;
% leaderCoord = [38.052326, -84.294367]; % GPS (lat,lon) coordinates at LMAC for origin of leader trjactory
% leaderCoord = [38.00629580000000, -84.405753700000000]; % My room
% leaderCoord = [38.0525005, -84.2941595]; % Deer Haven Park
leaderCoord = [38.0063086, -84.4057393];


% Pick the leader body-fixed yaw rotation rate
wg = 0.1;

global hFigure fCom u

% Create a figure for real time keypress
hFigure = figure('Name','NatNet UI','NumberTitle','off');
fCom = 0;

% Open a udp port to send messsages to UAV
% Need the Control systems toolbox for this to work
u = udp('192.168.2.255',5001);
fopen(u);

msg_id = 0;

tic;

while true
    t = toc;
    try
        set(hFigure,'KeyPressFcn',@myfun)
        disp(fCom)
    catch
        disp('Figure Closed.')
        disp('Ending Transmition.')
        break
    end
    
    
 
    
 
    
    
    [qg,pg,ug,attg] = qgfunc(t,wg);
%     leaderAtt = [;
    sendData(msg_id,fCom,qg,pg,ug,attg,leaderCoord)
    
    qg_prev = qg;
    pg_prev = pg;
    attg_prev = attg;
    
    pause(Ts)
end


fclose(u);
disp('Exit.')



%% Put the in-text functions here



function [qg,pg,ug,attg] = qgfunc(t,wg)
    

    % Non-moving leader
    qg = [0.0,0.0,-4.0];
    pg = [0.0,0.0,0.0];
    ug = [0.0,0.0,0.0];
    attg = [0.0,0.0,0.0];    

    % Translating Leader: Sinusoidal trajectory in the x direction
%     amp = 1;
%     freq = 1/50;
%     qg = [amp*sin(freq*2*pi*t),0.0,-4.0];
%     pg = [amp*freq*2*pi*cos(freq*2*pi*t),0.0,0.0];
%     ug = [-amp*4*(freq*pi)^2*sin(freq*2*pi*t),0.0,0.0];
%     attg = [0.0,0.0,0.0];


    % Rotating leader: Yaw based on constant yaw rate
    % On the to-do list
    
    

end

% Keystroke function
function myfun(src,event)
    global fCom
    keystroke = event.Key;
    switch keystroke
        case 't'
            fCom = 1;
            disp('Taking Off')
        case 'h'
            fCom = 2;
            disp('Hovering')
        case 'r'
            fCom = 3;
%             staticLeaderArg = 1;
            disp('RTL')
        case 'l'
            fCom = 4;
            disp('Landing')
        case 'v'
            fCom = 5;
            disp('Virtual Leader')
        case 'f'
            fCom = 6;
            disp('Formation Flight')
        otherwise
            disp('Try Again')
    end
end