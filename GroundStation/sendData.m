function sendData(msg_id,fCom,qg,pg,ug,leaderAtt,leaderCoord) 
% If multiple agents, uncomment the id piece above.

% qprime = q'/1000;
% pprime = p'/1000;
% qprime(3) = -qprime(3);
% pprime(3) = -pprime(3);
% theta(3) = -theta(3);


global u

% position.x = round(qprime(1),5);
% position.y = round(qprime(2),5);
% position.z = round(qprime(3),5);
% velocity.vx = round(pprime(1),5);
% velocity.vy = round(pprime(2),5);
% velocity.vz = round(pprime(3),5);
% attitude.roll = theta(1);
% attitude.pitch = theta(2);
% attitude.yaw = theta(3);
leader.qg = round(qg,5);
% leader.qgy = ;%round(leaderPos(2),5);
% leader.qgz = ;%ound(leaderPos(3),5);
leader.roll = leaderAtt(1);
leader.pitch = leaderAtt(2);
leader.yaw = leaderAtt(3);
leader.omega = [0.0,0.0,0.0];
leader.omega_dot = [0.0,0.0,0.0];
leader.pg = pg;
% leader.pgy = 0.0;
% leader.pgz = 0.0;
leader.ug = ug;
% leader.ugy = 0.0;
% leader.ugz = 0.0;
leader.lat = leaderCoord(1);
leader.lon = leaderCoord(2);

scooby.timestamp = 0.0;
scooby.ID = msg_id;
% scooby.position = position;
% scooby.velocity = velocity;
% scooby.attitude = attitude;
scooby.leader = leader;
scooby.flightSeq = fCom;
scooby.type = msg_id;

msg = jsonencode(scooby);

% Send the data to the network
try
   fwrite(u,msg)
%    clear msg
catch e
   fclose(u);
   delete(u);
   print('error')
   clear u
end
   
% Message = 'Message Sent';
% clear msg





end