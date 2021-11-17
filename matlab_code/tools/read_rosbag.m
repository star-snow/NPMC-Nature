%% Import rosbag

is_simulation = true;
bagname = '2020-05-28-16-25-22_out.bag';
nb_agents = 1;

log_folder = 'logs';
bag = rosbag(bagname);


%% Read poses

if is_simulation
    
    % Select gazebo pose topic
    pose_topic = strcat('/gazebo/model_states');
    bag_poses = select(bag, 'Topic', pose_topic);
    
    % Convert to time series and resample
    ts_poses = timeseries(bag_poses);
    
    
else
    
    for i = 1:nb_agents
        
        % Select optitrack position topic
        pose_topic = strcat('/vrpn_client_node/cf',num2str(i),'/pose');
        bag_poses(i) = select(bag, 'Topic', pose_topic);
        
        % Convert to time series and resample
        ts_poses(i) = timeseries(bag_poses(i));
        
        % Compute speed as position deriative
        ts_poses(i) = resample(ts_poses(i),timevec);
        ts_poses(i).Data(:,11) = [0; diff(ts_poses(i).Data(:,4))./diff(ts_poses(i).Time)*10^11];
        ts_poses(i).Data(:,12) = [0; diff(ts_poses(i).Data(:,5))./diff(ts_poses(i).Time)*10^11];
        ts_poses(i).Data(:,13) = [0; diff(ts_poses(i).Data(:,6))./diff(ts_poses(i).Time)*10^11];
        
        %     msgStructs = readMessages(bagselect_positions(i),'DataFormat','struct');
        %     msgStructs{1}
        %     pos_x = cellfun(@(m) double(m.Pose.Position.X),msgStructs);
        %     pos_y = cellfun(@(m) - double(m.Pose.Position.Y),msgStructs);
        %     pos_z = cellfun(@(m) double(m.Pose.Position.Z),msgStructs);
        
        % Get optitrack positions from the time series
        %     pos_x = ts_positions(i).Data(:,4);
        %     pos_y = - ts_positions(i).Data(:,5);
        %     pos_z = ts_positions(i).Data(:,6);
        
    end
    
end

%% Create position and velocity history

pos_history = [];
vel_history = [];

if is_simulation
    
    for i = 1:nb_agents
        % Add data to a table and change y orientation convention
        pos_history = [pos_history, ts_poses.Data(:,4), ...
            - ts_poses.Data(:,5), ts_poses.Data(:,6)];
        vel_history = [vel_history, ts_poses.Data(:,11), ...
            - ts_poses.Data(:,12), ts_poses.Data(:,13)];
    end
    
else
    
    for i = 1:nb_agents
        % Add data to a table and change y orientation convention
        pos_history = [pos_history, ts_poses(i).Data(:,4), ...
            - ts_poses(i).Data(:,5), ts_poses(i).Data(:,6)];
        vel_history = [vel_history, ts_poses(i).Data(:,11), ...
            - ts_poses(i).Data(:,12), ts_poses(i).Data(:,13)];
    end
end


% Write to workspace
workspace_name = extractBefore(bagname,".");
workspace_path = strcat(log_folder,'/',workspace_name);
save(workspace_path,'pos_history','vel_history');
