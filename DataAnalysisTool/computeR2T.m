function [desDist,rotation] = computeR2T(index,data,target_vectors)

    leader = struct;
    rotation = struct;
    desDist = struct;
    
    desDist.R2T_1 = zeros(3,length(index));
    desDist.R2T_2 = zeros(3,length(index));
    desDist.R2T_3 = zeros(3,length(index));
    desDist.R2T_dot_1 = zeros(3,length(index));
    desDist.R2T_dot_2 = zeros(3,length(index));
    desDist.R2T_dot_3 = zeros(3,length(index));
    desDist.R2T_ddot_1 = zeros(3,length(index));
    desDist.R2T_ddot_2 = zeros(3,length(index));
    desDist.R2T_ddot_3 = zeros(3,length(index));




   
try    
    for i = 1:length(index)
        % Set the leader trajectory
        leader(i).qg = [data.v1_leadXPos(index(i));data.v1_leadYPos(index(i));data.v1_leadZPos(index(i))];
        leader(i).pg = [data.v1_leadXVel(index(i));data.v1_leadYVel(index(i));data.v1_leadZVel(index(i))];
        
        rotation(i).Rx = [1 0 0; 0 cos(data.v1_leadRoll(index(i))) sin(data.v1_leadRoll(index(i))); 0 -sin(data.v1_leadRoll(index(i))) cos(data.v1_leadRoll(index(i)))];
        rotation(i).Ry = [cos(data.v1_leadPitch(index(i))) 0 -sin(data.v1_leadPitch(index(i))); 0 1 0; sin(data.v1_leadPitch(index(i))) 0 cos(data.v1_leadPitch(index(i)))];
        rotation(i).Rz = [cos(data.v1_leadYaw(index(i))) sin(data.v1_leadYaw(index(i))) 0; -sin(data.v1_leadYaw(index(i))) cos(data.v1_leadYaw(index(i))) 0; 0 0 1];

        rotation(i).Omega = [0 -data.v1_lead_wZ(index(i)) data.v1_lead_wY(index(i)); data.v1_lead_wZ(index(i)) 0 -data.v1_lead_wX(index(i)); -data.v1_lead_wY(index(i)) data.v1_lead_wX(index(i)) 0];
        
        rotation(i).Rg_I2B = rotation(i).Rx*rotation(i).Ry*rotation(i).Rz;
        rotation(i).Rg_dot = rotation(i).Rg_I2B'*rotation(i).Omega;
        rotation(i).Rg_ddot = rotation(i).Rg_I2B'*rotation(i).Omega*rotation(i).Omega; % Note this is for a constant angular velocity
        
        
        desDist.R2T_1(:,i) = leader(i).qg + rotation(i).Rg_I2B'*target_vectors(1,:)';
        desDist.R2T_2(:,i) = leader(i).qg + rotation(i).Rg_I2B'*target_vectors(2,:)';
        desDist.R2T_3(:,i) = leader(i).qg + rotation(i).Rg_I2B'*target_vectors(3,:)';
        
        desDist.R2T_dot_1(:,i) = leader(i).pg + rotation(i).Rg_dot*target_vectors(1,:)';
        desDist.R2T_dot_2(:,i) = leader(i).pg + rotation(i).Rg_dot*target_vectors(2,:)';
        desDist.R2T_dot_3(:,i) = leader(i).pg + rotation(i).Rg_dot*target_vectors(3,:)';
        
        desDist.R2T_ddot_1(:,i) = rotation(i).Rg_ddot*target_vectors(1,:)';
        desDist.R2T_ddot_2(:,i) = rotation(i).Rg_ddot*target_vectors(2,:)';
        desDist.R2T_ddot_3(:,i) = rotation(i).Rg_ddot*target_vectors(3,:)';
        
        
    end
catch
    1;
end
        


try
    for i = 1:length(index)
        % Set the leader trajectory
        leader(i).qg = [data.v2_leadXPos(index(i));data.v2_leadYPos(index(i));data.v2_leadZPos(index(i))];
        leader(i).pg = [data.v2_leadXVel(index(i));data.v2_leadYVel(index(i));data.v2_leadZVel(index(i))];
        
        rotation(i).Rx = [1 0 0; 0 cos(data.v2_leadRoll(index(i))) sin(data.v2_leadRoll(index(i))); 0 -sin(data.v2_leadRoll(index(i))) cos(data.v2_leadRoll(index(i)))];
        rotation(i).Ry = [cos(data.v2_leadPitch(index(i))) 0 -sin(data.v2_leadPitch(index(i))); 0 1 0; sin(data.v2_leadPitch(index(i))) 0 cos(data.v2_leadPitch(index(i)))];
        rotation(i).Rz = [cos(data.v2_leadYaw(index(i))) sin(data.v2_leadYaw(index(i))) 0; -sin(data.v2_leadYaw(index(i))) cos(data.v2_leadYaw(index(i))) 0; 0 0 1];

        rotation(i).Omega = [0 -data.v2_lead_wZ(index(i)) data.v2_lead_wY(index(i)); data.v2_lead_wZ(index(i)) 0 -data.v2_lead_wX(index(i)); -data.v2_lead_wY(index(i)) data.v2_lead_wX(index(i)) 0];
        
        rotation(i).Rg_I2B = rotation(i).Rx*rotation(i).Ry*rotation(i).Rz;
        rotation(i).Rg_dot = rotation(i).Rg_I2B'*rotation(i).Omega;
        
        
        desDist.R2T_1(:,i) = leader(i).qg + rotation(i).Rg_I2B'*target_vectors(1,:)';
        desDist.R2T_2(:,i) = leader(i).qg + rotation(i).Rg_I2B'*target_vectors(2,:)';
        desDist.R2T_3(:,i) = leader(i).qg + rotation(i).Rg_I2B'*target_vectors(3,:)';
        
        desDist.R2T_dot_1(:,i) = leader(i).pg + rotation(i).Rg_dot*target_vectors(1,:)';
        desDist.R2T_dot_2(:,i) = leader(i).pg + rotation(i).Rg_dot*target_vectors(2,:)';
        desDist.R2T_dot_3(:,i) = leader(i).pg + rotation(i).Rg_dot*target_vectors(3,:)';
    end
catch
    1;
end


try
    for i = 1:length(index)
        % Set the leader trajectory
        leader(i).qg = [data.v3_leadXPos(index(i));data.v3_leadYPos(index(i));data.v3_leadZPos(index(i))];
        leader(i).pg = [data.v3_leadXVel(index(i));data.v3_leadYVel(index(i));data.v3_leadZVel(index(i))];
        
        rotation(i).Rx = [1 0 0; 0 cos(data.v3_leadRoll(index(i))) sin(data.v3_leadRoll(index(i))); 0 -sin(data.v3_leadRoll(index(i))) cos(data.v3_leadRoll(index(i)))];
        rotation(i).Ry = [cos(data.v3_leadPitch(index(i))) 0 -sin(data.v3_leadPitch(index(i))); 0 1 0; sin(data.v3_leadPitch(index(i))) 0 cos(data.v3_leadPitch(index(i)))];
        rotation(i).Rz = [cos(data.v3_leadYaw(index(i))) sin(data.v3_leadYaw(index(i))) 0; -sin(data.v3_leadYaw(index(i))) cos(data.v3_leadYaw(index(i))) 0; 0 0 1];

        rotation(i).Omega = [0 -data.v3_lead_wZ(index(i)) data.v3_lead_wY(index(i)); data.v3_lead_wZ(index(i)) 0 -data.v3_lead_wX(index(i)); -data.v3_lead_wY(index(i)) data.v3_lead_wX(index(i)) 0];
        
        rotation(i).Rg_I2B = rotation(i).Rx*rotation(i).Ry*rotation(i).Rz;
        rotation(i).Rg_dot = rotation(i).Rg_I2B'*rotation(i).Omega;
        
        
        desDist.R2T_1(:,i) = leader(i).qg + rotation(i).Rg_I2B'*target_vectors(1,:)';
        desDist.R2T_2(:,i) = leader(i).qg + rotation(i).Rg_I2B'*target_vectors(2,:)';
        desDist.R2T_3(:,i) = leader(i).qg + rotation(i).Rg_I2B'*target_vectors(3,:)';
        
        desDist.R2T_dot_1(:,i) = leader(i).pg + rotation(i).Rg_dot*target_vectors(1,:)';
        desDist.R2T_dot_2(:,i) = leader(i).pg + rotation(i).Rg_dot*target_vectors(2,:)';
        desDist.R2T_dot_3(:,i) = leader(i).pg + rotation(i).Rg_dot*target_vectors(3,:)';
    end
catch
    1;
end


end