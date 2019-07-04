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
    
    % Plot the PWM=f(Position) function
    %pos = y(1,:);
    qV3 = y(1,:); %-51.56;
    qV2 = y(2,:);
    qV1 = y(3,:); %+24.32;
    pos = y(4,:); % mod(y(2,:)*180/pi,360);
    measVq = y(5,:);
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
        % get samples idxes for the present cluster
        posMap = ismember(posClusterIdxes,posClusterIdx);
        posSubset = pos(posMap);
        qIFilt(posClusterIdx).count = numel(posSubset);
        qIFilt(posClusterIdx).expIa = mean(qV1(posMap));
        qIFilt(posClusterIdx).expIb = mean(qV2(posMap));
        qIFilt(posClusterIdx).expIc = mean(qV3(posMap));
    end
    vIdx = [qIFilt.count]>0;
    
    % Plot the data
    Plotter.plot2dDataNfittedModel(...
        [],[System.toLatexInterpreterCompliant(joints{figIdx}) ' : quadrature PWM = f(Position)'],['motorMeasVq2pos-Fig' num2str(figIdx)],...
        pos,measVq,[],[],...
        'motor position (deg)','quadrature voltage (dutycycle)',...
        'Vq meas',[]);
    Plotter.plot2dDataNfittedModel(...
        [],[System.toLatexInterpreterCompliant(joints{figIdx}) ' : Phase "a" PWM = f(Position)'],['qV12pos-Fig' num2str(figIdx)],...
        pos,qV1,[qIFilt(vIdx).pos],[qIFilt(vIdx).expIa],...
        'motor position (deg)','phase PWM (dutycycle)',...
        'Ia meas','Ia filtered');
    Plotter.plot2dDataNfittedModel(...
        [],[System.toLatexInterpreterCompliant(joints{figIdx}) ' : Phase "b" PWM = f(Position)'],['qV22pos-Fig' num2str(figIdx)],...
        pos,qV2,[qIFilt(vIdx).pos],[qIFilt(vIdx).expIb],...
        'motor position (deg)','phase PWM (dutycycle)',...
        'Ib meas','Ib filtered');
    Plotter.plot2dDataNfittedModel(...
        [],[System.toLatexInterpreterCompliant(joints{figIdx}) ' : Phase "c" PWM = f(Position)'],['qV32pos-Fig' num2str(figIdx)],...
        pos,qV3,[qIFilt(vIdx).pos],[qIFilt(vIdx).expIc],...
        'motor position (deg)','phase PWM (dutycycle)',...
        'Ic meas','Ic filtered');
    
    %% Compute expected Vq, compensate the offsets and adjust the "reading axes" to obtain a constant Vq
    
    % Pre-compute all the Clark Park Trans. for the defined rotor positions
    CPT = ClarkParkTransform(pos);
    
    % Compute expected Vq without any compensation
    expVq = CPT.phase2dirquad(0,0,[qV1;qV2;qV3]);
    
    % Remove the PWM offsets and plot
    meanIa = mean([qIFilt(vIdx).expIa])
    meanIc = mean([qIFilt(vIdx).expIc])
    qV1_offComp = qV1-meanIa;
    qV3_offComp = qV3-meanIc;
    qV2_offComp = -qV1_offComp-qV3_offComp;
    
    expVqOffComp = CPT.phase2dirquad(0,0,[qV1_offComp;qV2_offComp;qV3_offComp]);
    
    % Plot expected Vq, compensated and not compensated
    Plotter.plot2dDataNfittedModel(...
        figuresHandler,[System.toLatexInterpreterCompliant(joints{figIdx}) ' : expected quadrature PWM = f(Position)'],['motorExpVq2pos-Fig-OffsetComp' num2str(figIdx)],...
        pos,expVq,[],[],...
        'motor position (deg)','quadrature PWM (dutycycle)',...
        'Vq exp',[]);
    
    Plotter.plot2dDataNfittedModel(...
        figuresHandler,[],['motorExpVq2pos-Fig-OffsetComp' num2str(figIdx)],...
        pos,expVqOffComp,[],[],...
        [],[],...
        'Vq exp offset compensation.',[]);

    % Compensate the phase shift errors only. Use an optimization approach to find the phase shifts that result in a
    % constant Vq.
    dPhi0 = [0 0]'; % Elec Phase Shift
    optimCtx = ClarkParkOptim(CPT,qV1,qV2,qV3);
    
    [dPhiOpt, motShift, resnorm, exitflag, output] = optimCtx.optimizeReadingAxes(dPhi0)
    
    expVqPhaseShiftComp = CPT.phase2dirquad(dPhiOpt(1),dPhiOpt(2),optimCtx.e1e2e3);
    
    % Plot expected Vq, compensated and not compensated
    Plotter.plot2dDataNfittedModel(...
        figuresHandler,[System.toLatexInterpreterCompliant(joints{figIdx}) ' : expected quadrature PWM = f(Position)'],['motorExpVq2pos-Fig-PhaseComp' num2str(figIdx)],...
        pos,expVq,[],[],...
        'motor position (deg)','quadrature PWM (dutycycle)',...
        'Vq exp',[]);
    
    Plotter.plot2dDataNfittedModel(...
        figuresHandler,[],['motorExpVq2pos-Fig-PhaseComp' num2str(figIdx)],...
        pos,expVqPhaseShiftComp,[],[],...
        [],[],...
        'Vq exp phase shift compensation',[]);
    
    % Compensate both offsets and phase shift errors.
    clear optimCtx;
    optimCtx = ClarkParkOptim(CPT,qV1_offComp,qV2_offComp,qV3_offComp);

    [dPhiOpt, motShift, resnorm, exitflag, output] = optimCtx.optimizeReadingAxes(dPhi0)
    
    expVqPhaseOffShiftComp = CPT.phase2dirquad(dPhiOpt(1),dPhiOpt(2),optimCtx.e1e2e3);
    
    % Plot expected Vq, compensated and not compensated
    Plotter.plot2dDataNfittedModel(...
        figuresHandler,[System.toLatexInterpreterCompliant(joints{figIdx}) ' : expected quadrature PWM = f(Position)'],['motorExpVq2pos-Fig-OffsetPhaseComp' num2str(figIdx)],...
        pos,expVq,[],[],...
        'motor position (deg)','quadrature PWM (dutycycle)',...
        'Vq exp',[]);
    
    Plotter.plot2dDataNfittedModel(...
        figuresHandler,[],['motorExpVq2pos-Fig-OffsetPhaseComp' num2str(figIdx)],...
        pos,expVqPhaseOffShiftComp,[],[],...
        [],[],...
        'Vq exp offset and phase shift compensation',[]);
    
end
    %% Emulate algorithm in 2FOC code
    
    [Va,Vb,Vc,Vd,Vq] = deal(zeros(size(pos)));
    Vdexp = repmat(40,size(pos)); % percent
    Vqexp = repmat(100,size(pos)); % percent
    
    for idx = 1:numel(pos)
        focEmul = FOCemulator();
        focEmul.updateState(round(pos(idx)*6));
        [Va(idx),Vb(idx),Vc(idx)] = focEmul.computePWMabc(Vdexp(idx),Vqexp(idx));
        [Vd(idx),Vq(idx)] = focEmul.computeIqId(Va(idx),Vc(idx));
    end
    
    % Plot emulated Va, Vb, Vc
    Plotter.plot2dDataNfittedModel(...
        figuresHandler,[System.toLatexInterpreterCompliant(joints{figIdx}) ' : 2FOC emulated Phase PWM voltages = f(Position)'],['motor2focVabc2pos-Fig-2FOCemul' num2str(figIdx)],...
        pos,Va,[],[],...
        'motor position (deg)','phase voltage PWM (dutycycle)',...
        'Va 2FOC',[]);
    
    Plotter.plot2dDataNfittedModel(...
        figuresHandler,[],['motor2focVabc2pos-Fig-2FOCemul' num2str(figIdx)],...
        pos,Vb,[],[],...
        [],[],...
        'Vb 2FOC',[]);
    
    Plotter.plot2dDataNfittedModel(...
        figuresHandler,[],['motor2focVabc2pos-Fig-2FOCemul' num2str(figIdx)],...
        pos,Vc,[],[],...
        [],[],...
        'Vc 2FOC',[]);
    
    Plotter.plot2dDataNfittedModel(...
        figuresHandler,[System.toLatexInterpreterCompliant(joints{figIdx}) ' : expected d-q PWM = f(Position)'],['motor2focVdq2pos-Fig-2FOCemul' num2str(figIdx)],...
        pos,Vdexp,[],[],...
        'motor position (deg)','phase voltage PWM (dutycycle)',...
        'Vd expected',[]);
    
    Plotter.plot2dDataNfittedModel(...
        figuresHandler,[],['motor2focVdq2pos-Fig-2FOCemul' num2str(figIdx)],...
        pos,Vqexp,[],[],...
        [],[],...
        'Vq expected',[]);
    
    Plotter.plot2dDataNfittedModel(...
        figuresHandler,[],['motor2focVdq2pos-Fig-2FOCemul' num2str(figIdx)],...
        pos,Vd/10,[],[],...
        [],[],...
        'Vd 2FOC',[]);
    
    Plotter.plot2dDataNfittedModel(...
        figuresHandler,[],['motor2focVdq2pos-Fig-2FOCemul' num2str(figIdx)],...
        pos,Vq/10,[],[],...
        [],[],...
        'Vq 2FOC',[]);
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
% 
    