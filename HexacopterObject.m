classdef UAV
    properties
        mass=1;
        momentum=ones(3,1);
        states=[32 56 234 2 2 45]';
        
    end
    methods
        
        function obj=UAV()
            % Class constructor
        end
        
        function rates=HexacopterModel(obj,inputs)
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
            
            
            rates(1:3)=(force./m)-skew_matrix*obj.states(1:3);
            rates(4)=(q*r*(momentum(2)-momentum(3))+torque(1))/momentum(1);
            rates(5)=(p*r*(momentum(3)-momentum(1))+torque(2))/momentum(2);
            rates(6)=(q*p*(momentum(1)-momentum(2))+torque(3))/momentum(3);
        end %rates
    end
    
end