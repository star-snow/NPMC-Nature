classdef Crazyflie < handle
    % Crazyflie: This class is creates and manages a Crazyflie drone.
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Crazyflie properties
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    properties
        
        id
        init

        pos_neu     % position in NEU frame, NEU: North-East-Up
        vel_neu     % velocity in NEU frame
        attitude    % attitude
        
        prev_pos_neu
        secs        % curent time in seconds, to compute the derivative
        prev_secs   % curent time in seconds, to compute the derivative
        
        pos_neu_history
        pos_cmd_history
        vel_neu_history
        vel_cmd_history

        % Publishers
        cmd_pos_pub
        cmd_vel_pub
        cmd_full_pub
        
        % Subscribers
        pos_sub
        vel_sub
        
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Crazyflie methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    methods
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function Crazyflie = Crazyflie(id, is_simulation, node)

            % Create a Crazyflie: assign id, publisher and subscriber
            Crazyflie.id = id;
            Crazyflie.init = false;
            
            Crazyflie.pos_neu_history = [];
            Crazyflie.pos_cmd_history = [];
            Crazyflie.vel_neu_history = [];
            
            cmd_pos_topic = strcat('/cf', num2str(Crazyflie.id), '/cmd_position');
            Crazyflie.cmd_pos_pub = ros.Publisher(node, ...
                cmd_pos_topic, 'crazyflie_driver/Position');
            
            cmd_velocity_topic = strcat('/cf', num2str(Crazyflie.id), '/cmd_velocity_world');
            Crazyflie.cmd_vel_pub = ros.Publisher(node, ...
                cmd_velocity_topic, 'crazyflie_driver/VelocityWorld');

            cmd_full_topic = strcat('/cf', num2str(Crazyflie.id), '/cmd_full_state');
            Crazyflie.cmd_full_pub = ros.Publisher(node, ...
                cmd_full_topic, 'crazyflie_driver/FullState');
            
            % Subscribe to local position
            if is_simulation
                % Simulation experiments on Gazebo
                pos_topic = strcat('/cf', num2str(Crazyflie.id), '/local_position');
                Crazyflie.pos_sub = ros.Subscriber(node, pos_topic, ...
                    'crazyflie_driver/GenericLogData', ...
                    @Crazyflie.local_position_callback);
            else
                % hardware experiments with Optitrack and Crazyswarm
                pos_topic = strcat('/vrpn_client_node','/cf', num2str(Crazyflie.id), '/pose');
                Crazyflie.pos_sub = ros.Subscriber(node, pos_topic, ...
                    'geometry_msgs/PoseStamped', ...
                    @Crazyflie.vrpn_pose_callback);
            end
                
            % Subscribe to velocity
            if is_simulation
                % Simulation experiments on Gazebo
                state_topic = strcat('/gazebo/model_states');
                Crazyflie.vel_sub = ros.Subscriber(node, state_topic, ...
                    'gazebo_msgs/ModelStates', ...
                    @Crazyflie.model_state_callback);
            else
                
            end

        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function delete(self)
            % delete - Delete the Crazyflie
            clear self.pos_sub self.vel_sub self.cmd_pos_pub
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function set_pos_history(self, position_neu)
            % set_pos_history - Set the position history of the drone in 
            % the NEU frame

            % Update pos_neu_history
            if isempty(self.pos_neu_history)
                self.pos_neu_history = position_neu';
            else
                self.pos_neu_history = [self.pos_neu_history; position_neu'];
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function set_pos_cmd_history(self, position_cmd)
            % set_pos_cmd_history - Set the position command history of 
            % the drone in the NEU frame

            % Update pos_neu_history
            if isempty(self.pos_cmd_history)
                self.pos_cmd_history = position_cmd';
            else
                self.pos_cmd_history = [self.pos_cmd_history; position_cmd'];
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function set_vel_cmd_history(self, velocity_cmd)
            % set_vel_cmd_history - Set the velocity command history of the 
            % drone in the NEU frame

            % Update vel_neu_history
            if isempty(self.vel_cmd_history)
                self.vel_cmd_history = velocity_cmd';
            else
                self.vel_cmd_history = [self.vel_cmd_history; velocity_cmd'];
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function set_vel_history(self, velocity_neu)
            % set_vel_history - Set the velocity of the drone in the NEU 
            % frame
            
            if isempty(self.vel_neu_history)
                self.vel_neu_history = velocity_neu';
            else
                self.vel_neu_history = [self.vel_neu_history; velocity_neu'];
            end
        end
        
    end
    
    methods (Access = private)
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function local_position_callback(self, ~, msg)
            % local_position_callback - callback for the Crazyflie
            % position in Gazebo
            self.pos_neu(1) = msg.Values(1);
            self.pos_neu(2) = msg.Values(2);
            self.pos_neu(3) = msg.Values(3);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function vrpn_pose_callback(self, ~, msg)
            % vrpn_pose_callback - callback for the drone position and
            % velocity for VRPN node with Optitrack
            if self.init
                self.prev_secs = self.secs;
                self.prev_pos_neu(1) = self.pos_neu(1);
                self.prev_pos_neu(2) = self.pos_neu(2);
                self.prev_pos_neu(3) = self.pos_neu(3);
            end
            
            self.secs = msg.Header.Stamp.Sec + msg.Header.Stamp.Nsec*10^-9;
            self.pos_neu(1) = msg.Pose.Position.X;
            self.pos_neu(2) = msg.Pose.Position.Y;
            self.pos_neu(3) = msg.Pose.Position.Z;
            
            if self.init
                
                self.vel_neu(1) = (self.pos_neu(1)-self.prev_pos_neu(1))/(self.secs-self.prev_secs);
                self.vel_neu(1) = (self.pos_neu(2)-self.prev_pos_neu(2))/(self.secs-self.prev_secs);
                self.vel_neu(1) = (self.pos_neu(3)-self.prev_pos_neu(3))/(self.secs-self.prev_secs);
            end
            
            self.init = true;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function model_state_callback(self, ~, msg)
            % model_state_callback - callback for the Crazyflie
            % velocity in Gazebo
            i = 1;
            while ~ strcmp(msg.Name{i},strcat('cf',num2str(self.id)))
                i = i + 1;
            end
            self.vel_neu(1) = msg.Twist(i).Linear.X;
            self.vel_neu(2) = msg.Twist(i).Linear.Y;
            self.vel_neu(3) = msg.Twist(i).Linear.Z;
        end

    end

end
