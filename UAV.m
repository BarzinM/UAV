classdef UAV
    
    properties
        mass=1;
        momentum=ones(3,1);
        states=[32 56 234 2 2 45]';
        roll; % phi
        pitch; % theta
        yaw;
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
    end % Methods
    
end