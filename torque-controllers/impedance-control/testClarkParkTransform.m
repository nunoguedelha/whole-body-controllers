clear variables

figure(1);
hold on;
pos = 0:0.01:60;
i1 = 800*cos((pos-9)*6*pi/180);
i3 = 800*cos((pos-51)*6*pi/180);
i2 = (-i1-i3);
plot(pos,i1);
plot(pos,i2);
plot(pos,i3);

% Pre-compute all the Clark Park Trans. for the defined rotor positions
CPT = ClarkParkTransform(pos);

aLine = [];

for degShift = [2;4]
    % Power conservative Clark transform
    phaseDelta = degShift*pi/180*6;
    i1i2i3(1,:) = i1;
    i1i2i3(2,:) = i2;
    i1i2i3(3,:) = i3;
    
    expIq = CPT.phase2dirquad(phaseDelta(1),phaseDelta(2),i1i2i3);
    
    oldLine = aLine;
    aLine=plot(pos,expIq);
    pause; disp(degShift);
    if ~isempty(oldLine)
        delete(oldLine);
    end
end

dPhi0 = zeros(2,1); % Elec Phase Shift
motShiftDeg = @(aElecPhaseShift) aElecPhaseShift/6*180/pi;
optimCtx = ClarkParkOptim(CPT,i1,i2,i3);

% Use an optimization approach to find the phase shifts that result in a constant Iq
[optimFunction,options] = ClarkParkOptim.getOptimConfig();

% optimize
%
% Important note:
% - Dq0 is the init vector for the optimization
% - Dq is the main optimization variable (format set by Dq0)
%
funcProps = functions(optimFunction);
funcName = funcProps.function;
switch funcName
    case 'lsqnonlin'
        [dPhiOpt, resnorm, ~, exitflag, output, ~] ...
            = optimFunction(@(dPhi) optimCtx.costFunction1(dPhi), ...
            dPhi0, [], [], options);
    otherwise
        % We are not computing optimalDq, but just using a previous
        % result for a performance evaluation
        error('Solver not supported !');
end

dPhiOpt
resnorm
exitflag
output

motShift = motShiftDeg(dPhiOpt);
display(motShift);

% Power conservative Clark transform
phaseDelta = motShift*pi/180*6;
expIq = CPT.phase2dirquad(phaseDelta(1),phaseDelta(2),i1i2i3);
figure(1);
aLine=plot(pos,expIq);

