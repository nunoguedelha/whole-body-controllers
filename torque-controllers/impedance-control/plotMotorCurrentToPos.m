clear variables

% joints = {'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll'};
joints = {'r_knee'};

% Power conservative Clark transform
phaseDelta1 = -0.4*pi/180*6;
phaseDelta2 = 2.4*pi/180*6;
ClarkT = 2*sqrt(1/3)*[
    1 cos(-2*pi/3+phaseDelta1) cos(-4*pi/3+phaseDelta2)
    0 sin(-2*pi/3+phaseDelta1) sin(-4*pi/3+phaseDelta2)
    1/sqrt(2) 1/sqrt(2) 1/sqrt(2)
    ];

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
    
    % Compute expected Iq
    
    % Rotation
    matElem = zeros(1,1,length(pos));
    [theta,vec0, vec1] = deal(matElem);
    theta(1,1,:) = pos/180*pi*6; % elec phase = 6 * motor position
    vec0(1,1,:) = 0;
    vec1(1,1,:) = 1;
    R = [
        cos(theta) -sin(theta) vec0
        sin(theta)  cos(theta) vec0
        vec0        vec0       vec1
        ];
    expIq = zeros(size(pos));
    
    for idx = 1:length(pos)
        qIaqIbqIc = [qIa(idx) qIb(idx) qIc(idx)]';
        expIdIqI0 = R(:,:,idx)*ClarkT*qIaqIbqIc;
        expIq(idx) = expIdIqI0(2);
    end
    
    % Plot expected Iq
    Plotter.plot2dDataNfittedModel(...
        [],[System.toLatexInterpreterCompliant(joints{figIdx}) ' : expected quadrature Current = f(Position)'],['motorExpIq2pos-Fig' num2str(figIdx)],...
        pos,expIq,[],[],...
        'motor position (deg)','quadrature current (mA)',...
        'Iq exp',[]);
    
    
    clear t; clear y;
end

