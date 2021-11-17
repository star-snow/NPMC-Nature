classdef Crazyswarm < handle
    % Crazyswarm : This class is meant to manage a set of Crazyflie drones.
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Crazyswarm properties:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    properties
        
        is_simulation
        nb_agents
        agents
        
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Crazyswarm methods:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    methods
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function Crazyswarm = Crazyswarm(is_simulation)
            % Crazyswarm - Create an empty Crazyswarm
            Crazyswarm.is_simulation = is_simulation;
            Crazyswarm.nb_agents = 0;
            Crazyswarm.agents = [];

        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function delete(self)
            % delete - Delete the Crazyswarm
            for id = 1:self.nb_agents
                self.agents(id).delete();
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function add(self, crazyflie)
            % add - Add a Crazyflie to the Crazyswarm
            self.nb_agents = self.nb_agents + 1;
            self.agents = [self.agents; crazyflie];
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function add_n(self, n, swarm_node)
            % add_n - Add n Crazyflie drones to the Crazyswarm
            % n: number of agents to add
            % node_swarm: node to which they are associated
            ids = self.nb_agents + 1:n;
            for id = ids
                self.add(Crazyflie(id, self.is_simulation, swarm_node));
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function all_positions = get_positions(self)
            % get_positions - Get Crazyflies positions in a matrix format
            % (row: x-y-z, col: agent_id)
            all_positions = zeros(3,self.nb_agents);
            for id = 1 : self.nb_agents
                all_positions(:,id) = self.agents(id).pos_neu;
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function all_velocities = get_velocities(self)
            % get_velocities - Get Crazyflies velocities in a matrix format
            % (row: x-y-z, col: agent_id)
            all_velocities = zeros(3,self.nb_agents);
            for id = 1 : self.nb_agents
                all_velocities(:,id) = self.agents(id).vel_neu;
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function all_states = get_states(self)
            % get_states - Get Crazyflies states (positions, velocities) 
            % in a vector format
            all_states = zeros(1,6*self.nb_agents);
            for id = 1 : self.nb_agents
                start_idx = (id-1)*3 + 1;
                all_states(start_idx:(start_idx+2)) = self.agents(id).pos_neu;
                start_idx = (3*self.nb_agents + (id-1)*3 + 1);
                all_states(start_idx:(start_idx+2)) = ...
                    self.agents(id).vel_neu;
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function takeoff(self, height, duration, rate)
            % takeoff - Make the Crazyswarm take off
            % height: height of the take off
            % duration: duration of the takeoff maneuver
            % rate: rate of publilcation of the lower-level position
            %       commands
            start_positions = self.get_positions();
            iter = ceil( duration * rate.DesiredRate );
            final_positions = start_positions + ...
                    [zeros(2, self.nb_agents); height*ones(1, self.nb_agents)];
            for i = 1 : iter
                current_positions = self.get_positions();
                for id = 1 : self.nb_agents
                    cf = self.agents(id);
                    msg = rosmessage('crazyflie_driver/Position');
                    msg.X = final_positions(1,id);
                    msg.Y = final_positions(2,id);
                    msg.Z = final_positions(3,id);
                    msg.Yaw = 0;
                    msg.Header.Seq = i;
                    msg.Header.Stamp = rostime('now','system');
                    send(cf.cmd_pos_pub,msg);
                    cf.set_pos_cmd_history(final_positions(:,id));
                    cf.set_pos_history(current_positions(:,id));
                end
                waitfor(rate);
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function land(self, duration, rate)
            % land - Make the Crazyswarm land
            % duration: duration of the takeoff maneuver
            % rate: rate of publication of the lower-level position
            %       commands
            start_positions = self.get_positions();
            iter = ceil( duration * rate.DesiredRate );
            land_positions = start_positions - ...
                    [zeros(2, self.nb_agents); start_positions(3,:)];
            for i = 1 : iter
                current_positions = self.get_positions();
                for id = 1 : self.nb_agents
                    cf = self.agents(id);
                    msg = rosmessage('crazyflie_driver/Position');
                    msg.X = land_positions(1,id);
                    msg.Y = land_positions(2,id);
                    msg.Z = land_positions(3,id);
                    msg.Yaw = 0;
                    msg.Header.Seq = i;
                    msg.Header.Stamp = rostime('now','system');
                    send(cf.cmd_pos_pub,msg);
                    cf.set_pos_cmd_history(land_positions(:,id));
                    cf.set_pos_history(current_positions(:,id));
                end
                waitfor(rate);
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function go_to(self, goal_positions, duration, rate)
            % go_to - Make the Crazyswarm go to specific positions.
            % goal_positions: positions to be reached, dim (3,N)
            % duration: duration of the takeoff maneuver
            % rate: rate of publication of the lower-level position
            %       commands
            start_positions = self.get_positions();
            iter = ceil( duration * rate.DesiredRate );
            distances = sqrt(sum((goal_positions - start_positions).^2,1));
            directions = (goal_positions - start_positions)./repmat(distances,3,1);
            deltas = (distances)/iter;
            for i = 1:iter
                next_positions = start_positions + ...
                    i*repmat(deltas,3,1).*directions;
                for id = 1 : self.nb_agents
                    cf = self.agents(id);
                    msg = rosmessage('crazyflie_driver/Position');
                    msg.X = next_positions(1,id);
                    msg.Y = next_positions(2,id);
                    msg.Z = next_positions(3,id);
                    msg.Yaw = 0;
                    msg.Header.Seq = i;
                    msg.Header.Stamp = rostime('now','system');
                    send(cf.cmd_pos_pub,msg);
                    cf.set_pos_cmd_history(next_positions(:,id));
                end
                waitfor(rate);
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function relative_go_to(self, goal_positions, duration, rate)
            start_positions = self.get_positions();
            iter = ceil( duration * rate.DesiredRate );
            distances = sqrt(sum((goal_positions).^2,2));
            deltas = (distances)/iter;
            for i = 1:iter
                next_positions = start_positions + ...
                    i*repmat(deltas,3,1).*directions;
                for id = 1 : self.nb_agents
                    cf = self.agents(id);
                    msg = rosmessage('crazyflie_driver/Position');
                    msg.X = next_positions(1,id);
                    msg.Y = next_positions(2,id);
                    msg.Z = next_positions(3,id);
                    msg.Yaw = 0;
                    msg.Header.Seq = i;
                    msg.Header.Stamp = rostime('now','system');
                    send(cf.cmd_pos_pub,msg);
                    cf.set_pos_cmd_history(next_positions(:,id));
                end
                waitfor(rate);
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function send_pos_commands(self, cmd_positions, rate)
            current_positions = self.get_positions();
            for id = 1 : self.nb_agents
                cf = self.agents(id);
                msg = rosmessage('crazyflie_driver/Position');
                msg.X = cmd_positions(1,id);
                msg.Y = cmd_positions(2,id);
                msg.Z = cmd_positions(3,id);
                msg.Yaw = 0;
                msg.Header.Seq = 1;
                msg.Header.Stamp = rostime('now','system');
                send(cf.cmd_pos_pub,msg);
                cf.set_pos_cmd_history(cmd_positions(:,id));
                cf.set_pos_history(current_positions(:,id));
            end
            waitfor(rate);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function send_vel_commands(self, cmd_velocities, rate)
            current_velocities = self.get_velocities();
            for id = 1 : self.nb_agents
                cf = self.agents(id);
                msg = rosmessage('crazyflie_driver/VelocityWorld');
                msg.Vel.X = cmd_velocities(1,id);
                msg.Vel.Y = cmd_velocities(2,id);
                msg.Vel.Z = cmd_velocities(3,id);
                msg.YawRate = 0;
                msg.Header.Seq = 1;
                msg.Header.Stamp = rostime('now','system');
                send(cf.cmd_vel_pub,msg);
                cf.set_vel_cmd_history(cmd_velocities(:,id));
                cf.set_vel_history(current_velocities(:,id));
            end
            waitfor(rate);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function send_full_commands(self, cmd_positions, ...
                cmd_velocities, cmd_accelerations, rate)
            current_positions = self.get_positions();
            for id = 1 : self.nb_agents
                cf = self.agents(id);
                msg = rosmessage('crazyflie_driver/FullState');
                msg.Pose.Position.X = cmd_positions(1,id);
                msg.Pose.Position.Y = cmd_positions(2,id);
                msg.Pose.Position.Z = cmd_positions(3,id);
                msg.Twist.Linear.X = cmd_velocities(1,id);
                msg.Twist.Linear.Y = cmd_velocities(2,id);
                msg.Twist.Linear.Z = cmd_velocities(3,id);
                msg.Acc.X = cmd_accelerations(1,id);
                msg.Acc.Y = cmd_accelerations(2,id);
                msg.Acc.Z = cmd_accelerations(3,id);
                % Avoid filling the header for speeding up the filling
                % of the message.
                % msg.Header.Seq = 1;
                % msg.Header.Stamp = rostime('now','system');
                send(cf.cmd_full_pub,msg);
                cf.set_pos_cmd_history(cmd_positions(:,id));
                cf.set_pos_history(current_positions(:,id));
            end
            waitfor(rate);
        end
        
    end
    
    methods (Access = private)
        

    end

end
