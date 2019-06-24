clear variables

% joints = {'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll'};
joints = {'r_knee'};

% For each figure
for figIdx=1
    % Get the data
    figure(figIdx);
    h=gca;
    axesObjs=h.Children;
    
    for axesIdx = 1:5
        switch class(axesObjs(axesIdx))
            case {'matlab.graphics.chart.primitive.Scatter','matlab.graphics.chart.primitive.Stair','matlab.graphics.primitive.Line'}
                t = axesObjs(axesIdx).XData;
                y(axesIdx,:) = axesObjs(axesIdx).YData;
                
            case 'matlab.graphics.chart.primitive.animatedline'
                [t,y(axesIdx,:)]=getpoints(axesObjs(2));
            otherwise
        end
    end
    
    disp(axesObjs);
    
    % Plot the Current=f(Position) function
    %pos = y(1,:);
    qIc = y(1,:); %-51.56;
    qIb = y(2,:);
    qIa = y(3,:); %+24.32;
    pos = y(4,:); % mod(y(2,:)*180/pi,360);
    measIq = y(5,:);
    
    Plotter.plot2dDataNfittedModel(...
        [],[System.toLatexInterpreterCompliant(joints{figIdx}) ' : quadrature Current = f(Position)'],['motorMeasIq2pos-Fig' num2str(figIdx)],...
        pos,measIq,[],[],...
        'motor position (deg)','quadrature current (mA)',...
        'Iq meas',[]);
    Plotter.plot2dDataNfittedModel(...
        [],[System.toLatexInterpreterCompliant(joints{figIdx}) ' : Phase "a" Current = f(Position)'],['qIa2pos-Fig' num2str(figIdx)],...
        pos,qIa,[],[],...
        'motor position (deg)','phase current (mA)',...
        'Ia meas',[]);
    Plotter.plot2dDataNfittedModel(...
        [],[System.toLatexInterpreterCompliant(joints{figIdx}) ' : Phase "b" Current = f(Position)'],['qIb2pos-Fig' num2str(figIdx)],...
        pos,qIb,[],[],...
        'motor position (deg)','phase current (mA)',...
        'Ib meas',[]);
    Plotter.plot2dDataNfittedModel(...
        [],[System.toLatexInterpreterCompliant(joints{figIdx}) ' : Phase "c" Current = f(Position)'],['qIc2pos-Fig' num2str(figIdx)],...
        pos,qIc,[],[],...
        'motor position (deg)','phase current (mA)',...
        'Ic meas',[]);
    
    % Compute expected Iq and adjust the "reading axes" to obbtain a constant Iq
    
    % Pre-compute all the Clark Park Trans. for the defined rotor positions
    CPT = ClarkParkTransform(pos);
    
    dPhi0 = zeros(2,1); % Elec Phase Shift
    optimCtx = ClarkParkOptim(CPT,qIa,qIb,qIc);
    
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
    
    motShift = CPT.elec2rotorPhase(dPhiOpt,6)*180/pi;
    display(motShift);
    
    % Re-compute Ic with the optimized values
    expIq = CPT.phase2dirquad(dPhiOpt(1),dPhiOpt(2),optimCtx.i1i2i3);
    
    % Plot expected Iq
    Plotter.plot2dDataNfittedModel(...
        [],[System.toLatexInterpreterCompliant(joints{figIdx}) ' : expected quadrature Current = f(Position)'],['motorExpIq2pos-Fig' num2str(figIdx)],...
        pos,expIq,[],[],...
        'motor position (deg)','quadrature current (mA)',...
        'Iq exp',[]);
    
    
    clear t; clear y;
end

