classdef ClarkParkOptim < handle
    %UNTITLED5 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (GetAccess=public, SetAccess=protected)
        CPT@ClarkParkTransform;
        i1i2i3@double;
    end
    
    methods
        function obj = ClarkParkOptim(CPT,i1,i2,i3)
            %UNTITLED5 Construct an instance of this class
            %   Detailed explanation goes here
            obj.CPT = CPT;
            obj.i1i2i3(1,:) = i1;
            obj.i1i2i3(2,:) = i2;
            obj.i1i2i3(3,:) = i3;
        end
        
        function errorVec = costFunction1(obj, dPhi)
            Iq = obj.CPT.phase2dirquad(dPhi(1),dPhi(2),obj.i1i2i3);
            errorVec = Iq-mean(Iq);
        end
    end
    
    methods (Static=true)
        [optimFunc,options] = getOptimConfig();
    end
end

