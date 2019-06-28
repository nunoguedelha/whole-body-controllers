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
        
        function [dPhiOpt, motShift, resnorm, exitflag, output] = optimizeReadingAxes(obj, dPhi0)
            % Use an optimization approach to find the phase shifts that result in a constant Iq
            [optimFunction,options] = ClarkParkOptim.getOptimConfig();
            
            % optimize
            %
            % Important note:
            % - dPhi0 is the init vector for the optimization
            % - dPhi is the main optimization variable (format set by dPhi0)
            %
            funcProps = functions(optimFunction);
            funcName = funcProps.function;
            switch funcName
                case 'lsqnonlin'
                    [dPhiOpt, resnorm, ~, exitflag, output, ~] ...
                        = optimFunction(@(dPhi) obj.costFunction1(dPhi), ...
                        dPhi0, [], [], options);
                otherwise
                    % We are not computing optimalDq, but just using a previous
                    % result for a performance evaluation
                    error('Solver not supported !');
            end
            motShift = obj.CPT.elec2rotorPhase(dPhiOpt,6)*180/pi;
        end
    end
    
    methods (Static=true)
        [optimFunc,options] = getOptimConfig();
    end
end

