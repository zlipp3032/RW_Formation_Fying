



clc
clear
close all


Ts = 0.055;
% leaderCoord = [38.052326, -84.294367]; % GPS (lat,lon) coordinates at LMAC for origin of leader trjactory
leaderCoord = [38.006309500000000, -84.405756400000000]; % My room


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
    
    [qg,pg,ug] = qgfunc(t);
    leaderAtt = [0.0,0.0,0.0];
    sendData(msg_id,fCom,qg,pg,ug,leaderAtt,leaderCoord)
    
    pause(Ts)
end


fclose(u);
disp('Exit.')



%% Put the in-text functions here



function [qg,pg,ug] = qgfunc(t)
    
    qg = [sin(t),0.0,-4.0];
    pg = [cos(t),0.0,0.0];
    ug = [-sin(t),0.0,0.0];
    

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