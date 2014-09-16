classdef UAV
    
    properties
        mass=1;
        position = [-5, 0, 0]';
        orientation = [0, 0, 0]';
        
        momentum=ones(3,1);
        states=[0 0 0 0 0 0]';
%         roll; % phi
%         pitch; % theta
%         yaw;
        trust_constant=1;
        CrossSectionArea=[40 40 10000]'/10000;
        C=1;
        
        
    end % Properties
    methods
        
        function obj=UAV()
            % Class constructor
        end % UAV class constructor
        
        function rates=hexacopterModel(obj,inputs)
            %% Hexacopter system modelling
            %
            %   states: [u v w p q r]' where u, v, w are linear velocities and p, q,
            %          and r are rotational velocities
            %
            %   inputs: [force torque]'
            %
            %
            
            force=inputs(1:3);
            torque=inputs(4:end);
            %% Initialization
            % number_of_states=length(states);
            % number_of_inputs=length(inputs);
            
            rates=zeros(size(obj.states));
            
            [u,v,w,p,q,r]=deal(obj.states(1),obj.states(2),obj.states(3),...
                obj.states(4),obj.states(5),obj.states(6));
            
            skew_matrix=[0,-r,q;r,0,-p;-q,p,0];
            
            
            rates(1:3)=(force./obj.mass)-skew_matrix*obj.states(1:3);
            rates(4)=(q*r*(obj.momentum(2)-obj.momentum(3))+torque(1))/obj.momentum(1);
            rates(5)=(p*r*(obj.momentum(3)-obj.momentum(1))+torque(2))/obj.momentum(2);
            rates(6)=(q*p*(obj.momentum(1)-obj.momentum(2))+torque(3))/obj.momentum(3);
        end % HexacopterModel
        
        function forces=forceCalculation()
            gg=9.8;
            
            gravity=obj.mass*gg*[sin(obj.pitch),cos(obj.pitch)*sin(obj.roll),...
                cos(obj.pitch)*cos(obj.roll)]';
            
            thrust_vector=[0,0,trust_value]';
            
            rotor_drag=-obj.mu*[obj.state(1),obj.state(2),0]';
            
            air_resistance=-[obj.cross_section_area.*obj.states(1:3).*abs(obj.states(1:3))]'*obj.C/2;
            
            forces=gravity+thrust_vector+rotor_drag+air_resistance;
            
        end % ForceCalculation
        
        function torque=torqueCalculation()
            
            yaw_counter_torque=[0,0,obj.propeller_inertia*obj.propeller_speed_change_rate]';
            
            gyroscopic_effect=obj.propeller_inertia*obj.overall_propeller_speed*[]'; % ?????
            
        end % TorqueCalculation
        
        function obj = simulateUAV(obj, inputs, timestep)

            acceleration =  hexacopterModel(obj, inputs)
            obj.position = (acceleration(1:3) * timestep^2)/2 + obj.states(1:3) * timestep + obj.position
            obj.position(1)
            obj.position(2)
            obj.orientation = (acceleration(4:6) * timestep^2)/2 + obj.states(4:6) * timestep + obj.orientation;
            obj.states=acceleration*timestep+obj.states;
        end % movement function
        
        function R_overall = rotation(obj)
            
            roll = obj.orientation(1);
            pitch = obj.orientation(2);
            yaw = obj.orientation(3);
            
            RX = [1, 0, 0;
                0, cos(roll), -sin(roll);
                0, sin(roll), cos(roll)];
            RY = [cos(pitch), 0, sin(pitch);
                0, 1, 0;
                -sin(pitch), 0, cos(pitch)];
            RZ = [cos(yaw), -sin(yaw), 0;
                sin(yaw), cos(yaw), 0;
                0, 0, 1];
            
            R_overall = RZ * RY * RX;
        end
        
        function showUAV(obj)
            radius = 1;
            degrees=pi*[0:60:360] / 180;
            x_geometrics = radius * sin(degrees);
            y_geometrics = radius * cos(degrees);
            axis_length=1;
            
            x = obj.position(1)
            y = obj.position(2)
            z = obj.position(3)
            
            vector = rotation(obj) * [axis_length, 0, 0]';
            
            u = axis_length * vector(1) + x
            v = axis_length * vector(2) + y
            w = axis_length * vector(3) + z
            
            body = rotation(obj) * [x_geometrics; y_geometrics; zeros(size(x_geometrics))];
            size(body)
            
            body(1,:) = body(1,:) + x;
            body(2,:) = body(2,:) + y;
            body(3,:) = body(3,:) + z;
            body
            
            
            plot3(body(1,:), body(2,:), body(3,:),'--','LineWidth',3,'MarkerSize',20)
            hold on
            plot3([body(1,6),body(1,1)], [body(2,6),body(2,1)], [body(3,6),body(3,1)],'r*','LineWidth',3,'MarkerSize',10)
            
        end % showHexa function
    end % Methods
    
end