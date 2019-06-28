classdef ClarkParkOptim < handle
    %UNTITLED5 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (GetAccess=public, SetAccess=protected)
        CPT@ClarkParkTransform;
        e1e2e3@double;
    end
    
    methods
        function obj = ClarkParkOptim(CPT,e1,e2,e3)
            %UNTITLED5 Construct an instance of this class
            %   Detailed explanation goes here
            obj.CPT = CPT;
            obj.e1e2e3(1,:) = e1;
            obj.e1e2e3(2,:) = e2;
            obj.e1e2e3(3,:) = e3;
        end
        
        function errorVec = costFunction1(obj, dPhi)
            Iq = obj.CPT.phase2dirquad(dPhi(1),dPhi(2),obj.e1e2e3);
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

