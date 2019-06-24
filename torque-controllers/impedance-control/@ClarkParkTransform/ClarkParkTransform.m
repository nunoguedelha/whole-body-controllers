classdef ClarkParkTransform < handle
    %ClarkParkTransform Pre-computes the transform values for a series of rotor angles
    %   Detailed explanation goes here
    
    properties
        pos@double;
        clarkT@function_handle;
        parkR@double;
    end
    
    methods
        function obj = ClarkParkTransform(pos)
            % Set rotor positions
            obj.pos = pos;
            
            % Define the Clark transform
            obj.clarkT = @(dPhiAxis2,dPhiAxis3) sqrt(2/3)*[
                1 cos(-2*pi/3+dPhiAxis2) cos(-4*pi/3+dPhiAxis3)
                0 sin(-2*pi/3+dPhiAxis2) sin(-4*pi/3+dPhiAxis3)
                1/sqrt(2) 1/sqrt(2) 1/sqrt(2)
                ];
            
            % Define the Park Rotation
            matElem = zeros(1,1,length(pos));
            [theta,vec0, vec1] = deal(matElem);
            theta(1,1,:) = pos/180*pi*6;
            vec0(1,1,:) = 0;
            vec1(1,1,:) = 1;
            obj.parkR = [
                cos(theta) -sin(theta) vec0
                sin(theta)  cos(theta) vec0
                vec0        vec0       vec1
                ];
        end
        
        function Iq = phase2dirquad(obj,dPhiAxis2,dPhiAxis3,i1i2i3)
            %clarkParkT Computes the full Clark-Park Trans.
            Iq = zeros(size(obj.pos));
            for idx = 1:length(obj.pos)
                IdIqI0 = obj.parkR(:,:,idx)*obj.clarkT(dPhiAxis2,dPhiAxis3)*i1i2i3(:,idx);
                Iq(idx) = IdIqI0(2);
            end
        end
    end
end

