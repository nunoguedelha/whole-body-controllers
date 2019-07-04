clear variables

% joints = {'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll'};
joints = {'r_hip_pitch'};

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
    clear t; clear y;
    
    % Prepare figure handler
    figuresHandler = DiagPlotFiguresHandler('figs');
    
    
if 0
    % Compute averaged samples
    clusterSize = 10;
    posClusters = (0:60*clusterSize)/clusterSize;
    qIFilt = struct(...
        'count',cell(size(posClusters)),...
        'pos',num2cell(posClusters),...
        'expIa',cell(size(posClusters)),...
        'expIb',cell(size(posClusters)),...
        'expIc',cell(size(posClusters))...
        ); % define clusters
    posClusterIdxes = 1 + round(pos*clusterSize); % cluster indexes
    
    for posClusterIdx = 1:numel(qIFilt)
        % get samples idxes for the current cluster
        posMap = ismember(posClusterIdxes,posClusterIdx);
        posSubset = pos(posMap);
        qIFilt(posClusterIdx).count = numel(posSubset);
        qIFilt(posClusterIdx).expIa = mean(qIa(posMap));
        qIFilt(posClusterIdx).expIb = mean(qIb(posMap));
        qIFilt(posClusterIdx).expIc = mean(qIc(posMap));
    end
    vIdx = [qIFilt.count]>0;
    
    % Plot the data
    Plotter.plot2dDataNfittedModel(...
        [],[System.toLatexInterpreterCompliant(joints{figIdx}) ' : quadrature Current = f(Position)'],['motorMeasIq2pos-Fig' num2str(figIdx)],...
        pos,measIq,[],[],...
        'motor position (deg)','quadrature current (mA)',...
        'Iq meas',[]);
    Plotter.plot2dDataNfittedModel(...
        [],[System.toLatexInterpreterCompliant(joints{figIdx}) ' : Phase "a" Current = f(Position)'],['qIa2pos-Fig' num2str(figIdx)],...
        pos,qIa,[qIFilt(vIdx).pos],[qIFilt(vIdx).expIa],...
        'motor position (deg)','phase current (mA)',...
        'Ia meas','Ia filtered');
    Plotter.plot2dDataNfittedModel(...
        [],[System.toLatexInterpreterCompliant(joints{figIdx}) ' : Phase "b" Current = f(Position)'],['qIb2pos-Fig' num2str(figIdx)],...
        pos,qIb,[qIFilt(vIdx).pos],[qIFilt(vIdx).expIb],...
        'motor position (deg)','phase current (mA)',...
        'Ib meas','Ib filtered');
    Plotter.plot2dDataNfittedModel(...
        [],[System.toLatexInterpreterCompliant(joints{figIdx}) ' : Phase "c" Current = f(Position)'],['qIc2pos-Fig' num2str(figIdx)],...
        pos,qIc,[qIFilt(vIdx).pos],[qIFilt(vIdx).expIc],...
        'motor position (deg)','phase current (mA)',...
        'Ic meas','Ic filtered');
    
    %% Compute expected Iq, compensate the offsets and adjust the "reading axes" to obtain a constant Iq
    
    % Pre-compute all the Clark Park Trans. for the defined rotor positions
    CPT = ClarkParkTransform(pos);
    
    % Compute expected Iq without any compensation
    expIq = CPT.phase2dirquad(0,0,[qIa;qIb;qIc]);
    
    % Remove the current offsets and plot
    meanIa = mean([qIFilt(vIdx).expIa])
    meanIc = mean([qIFilt(vIdx).expIc])
    qIa_offComp = qIa-meanIa;
    qIc_offComp = qIc-meanIc;
    qIb_offComp = -qIa_offComp-qIc_offComp;
    
    expIqOffComp = CPT.phase2dirquad(0,0,[qIa_offComp;qIb_offComp;qIc_offComp]);
    
    % Plot expected Iq, compensated and not compensated
    Plotter.plot2dDataNfittedModel(...
        figuresHandler,[System.toLatexInterpreterCompliant(joints{figIdx}) ' : expected quadrature Current = f(Position)'],['motorExpIq2pos-Fig-OffsetComp' num2str(figIdx)],...
        pos,expIq,[],[],...
        'motor position (deg)','quadrature current (mA)',...
        'Iq exp',[]);
    
    Plotter.plot2dDataNfittedModel(...
        figuresHandler,[],['motorExpIq2pos-Fig-OffsetComp' num2str(figIdx)],...
        pos,expIqOffComp,[],[],...
        [],[],...
        'Iq exp offset compensation.',[]);

    % Compensate the phase shift errors only. Use an optimization approach to find the phase shifts that result in a
    % constant Iq.
    dPhi0 = [0 0]'; % Elec Phase Shift
    optimCtx = ClarkParkOptim(CPT,qIa,qIb,qIc);
    
    [dPhiOpt, motShift, resnorm, exitflag, output] = optimCtx.optimizeReadingAxes(dPhi0)
    
    expIqPhaseShiftComp = CPT.phase2dirquad(dPhiOpt(1),dPhiOpt(2),optimCtx.e1e2e3);
    
    % Plot expected Iq, compensated and not compensated
    Plotter.plot2dDataNfittedModel(...
        figuresHandler,[System.toLatexInterpreterCompliant(joints{figIdx}) ' : expected quadrature Current = f(Position)'],['motorExpIq2pos-Fig-PhaseComp' num2str(figIdx)],...
        pos,expIq,[],[],...
        'motor position (deg)','quadrature current (mA)',...
        'Iq exp',[]);
    
    Plotter.plot2dDataNfittedModel(...
        figuresHandler,[],['motorExpIq2pos-Fig-PhaseComp' num2str(figIdx)],...
        pos,expIqPhaseShiftComp,[],[],...
        [],[],...
        'Iq exp phase shift compensation',[]);
    
    % Compensate both offsets and phase shift errors.
    clear optimCtx;
    optimCtx = ClarkParkOptim(CPT,qIa_offComp,qIb_offComp,qIc_offComp);

    [dPhiOpt, motShift, resnorm, exitflag, output] = optimCtx.optimizeReadingAxes(dPhi0)
    
    expIqPhaseOffShiftComp = CPT.phase2dirquad(dPhiOpt(1),dPhiOpt(2),optimCtx.e1e2e3);
    
    % Plot expected Iq, compensated and not compensated
    Plotter.plot2dDataNfittedModel(...
        figuresHandler,[System.toLatexInterpreterCompliant(joints{figIdx}) ' : expected quadrature Current = f(Position)'],['motorExpIq2pos-Fig-OffsetPhaseComp' num2str(figIdx)],...
        pos,expIq,[],[],...
        'motor position (deg)','quadrature current (mA)',...
        'Iq exp',[]);
    
    Plotter.plot2dDataNfittedModel(...
        figuresHandler,[],['motorExpIq2pos-Fig-OffsetPhaseComp' num2str(figIdx)],...
        pos,expIqPhaseOffShiftComp,[],[],...
        [],[],...
        'Iq exp offset and phase shift compensation',[]);
    
end
    %% Emulate algorithm in 2FOC code
    
    Id = zeros(size(pos));
    Iq = zeros(size(pos));
    
    for idx = 1:numel(pos)
        focEmul = FOCemulator();
        focEmul.updateState(round(pos(idx)*6));
        [Id(idx),Iq(idx)] = focEmul.computeIqId(qIa(idx),qIc(idx));
    end
    
    % Plot emulated Iq
    Plotter.plot2dDataNfittedModel(...
        figuresHandler,[System.toLatexInterpreterCompliant(joints{figIdx}) ' : 2FOC emulated quadrature Current = f(Position)'],['motor2focIq2pos-Fig-2FOCemul' num2str(figIdx)],...
        pos,Iq,[],[],...
        'motor position (deg)','quadrature current (mA)',...
        'Iq 2FOC',[]);
    
end

%% Converting the cos, sin tables in the 2FOC code

% % Original tables
% cos_table = [32767,32763,32748,32723,32688,32643,32588,32523,32449,32364,32270,32165,32051,31928,31794,31651,31498,31336,31164,30982,30791,30591,30381,30163,29935,29697,29451,29196,28932,28659,28377];
% sin_table = [    0,  330,  660,  990, 1319, 1648, 1977, 2305, 2632, 2959, 3285, 3609, 3933, 4255, 4576, 4896, 5214, 5531, 5846, 6159, 6470, 6779, 7087, 7392, 7694, 7995, 8293, 8588, 8881, 9171, 9459];
% 
% % computed tables
% cos_tbl = round(cos((0:1:30)/180*pi)*32767);
% sin_tbl = round(sin((0:1:30)/180*pi)*32767/sqrt(3));
% 
% % Identified phase shift errors
% dPhi = [0.0324 0.3422];
% 
% % Since the Clark-Park transform is:
% % clarkT = @(dPhi1,dPhi2) sqrt(2/3)*[
% %     1 cos(-2*pi/3+dPhi1) cos(-4*pi/3+dPhi2)
% %     0 sin(-2*pi/3+dPhi1) sin(-4*pi/3+dPhi2)
% %     1/sqrt(2) 1/sqrt(2) 1/sqrt(2)
% %     ];
% % 
% % We have to substract the identified dPhi from the cos/sin arguments in the tables
% 
% % converted tables
% cos_table_dphi1 = round(cos((0:1:30)/180*pi-dPhi(1))*32767);
% sin_table_dphi1 = round(sin((0:1:30)/180*pi-dPhi(1))*32767/sqrt(3));
% 
% cos_table_dphi2 = round(cos((0:1:30)/180*pi-dPhi(2))*32767);
% sin_table_dphi2 = round(sin((0:1:30)/180*pi-dPhi(2))*32767/sqrt(3));
% 
% clarkT = @(dPhiAxis2,dPhiAxis3) sqrt(2/3)*[
%     1 cos(-2*pi/3+dPhiAxis2) cos(-4*pi/3+dPhiAxis3)
%     0 sin(-2*pi/3+dPhiAxis2) sin(-4*pi/3+dPhiAxis3)
%     1/sqrt(2) 1/sqrt(2) 1/sqrt(2)
%     ];
% 
% clarkTcorr = clarkT(dPhi(1),dPhi(2))./clarkT(0,0);
