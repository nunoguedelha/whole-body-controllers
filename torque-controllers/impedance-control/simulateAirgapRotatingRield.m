% Simulate air-gap air-gap field

close all
clear variables

% define phase currents
I = 20; % current amplitude
p = 6; % number of paires of poles
% phase inductances
Lu = 0.250; % mH
Lv = 0.250; % mH
Lw = 0.250; % mH
% rotor velocity
wr = 8*pi; % rad/s
% the location where we measure the magnetic flux
thetaPhi = (0:(1/p):360);
% The rotor positions to simulate
dTheta = (1/p);
thetaR = (0:dTheta:360);
% The Direct-Quadrature-zero vector
i_dq0 = repmat([0 1 0]',1,length(thetaR));

% Create the rotating field
rotatingField = AirGapRotatingField(p,Lu,Lv,Lw,thetaPhi*pi/180);

% Pre-compute all the quantities for the defined rotor positions
CPT = ClarkParkTransform(thetaR);     % Clark Park Transforms
i_uvw = CPT.dirquad2phase(0,0,i_dq0); % phase U, V, W currents
dT = dTheta/wr;                       % compute the time step

% plot the magnetic flux resulting from all 3 phases for each reached rotor position
stopLoop = System.Buffer(false);
figure('Name','Air-gap rotating field','WindowKeyPressFcn',{@stopPlot,stopLoop});
hold on;

maxPHI = max(rotatingField.computePHI_uvw(i_uvw(:,1)));
idx = 0;
xLineFrom = [rotatingField.get('readingAxesPosSeq').pos]*180/pi/p;
xLineTo = xLineFrom;
yLineFrom = -1.5*maxPHI*ones(size(xLineFrom));
yLineTo = 1.5*maxPHI*ones(size(xLineFrom));
line(...
    [xLineFrom;xLineTo],...
    [yLineFrom;yLineTo],...
    'color','k','linestyle',':'); % plot the reading axes
text(xLineTo,yLineTo,{rotatingField.get('readingAxesPosSeq').seq}); % add the reading axes labels
rotorPos = (0:(pi/p):(2*pi))*180/pi;
rotorPosNoffset = rotorPos(1:2:end);
rotorPosSoffset = rotorPos(2:2:end);

prevPhiLine = [];
prevRotorNline = [];
prevRotorSline = [];

while ~stopLoop.value
    idx = mod(idx,length(thetaR))+1;
    % Compute the resulting air-gap magnetic flux for this rotor position
    PHI_uvw = rotatingField.computePHI_uvw(i_uvw(:,idx));
    rotorPosN = mod(thetaR(idx) + rotorPosNoffset,360);
    rotorPosS = mod(thetaR(idx) + rotorPosSoffset,360);
    % Plot
    phiLine = plot(thetaPhi,PHI_uvw,'g');
    rotorPline = line(...
        [rotorPosN;rotorPosN],...
        [-maxPHI*ones(size(rotorPosN));maxPHI*ones(size(rotorPosN))],...
        'color','b','Marker','^');
    rotorNline = line(...
        [rotorPosS;rotorPosS],...
        [maxPHI*ones(size(rotorPosS));-maxPHI*ones(size(rotorPosS))],...
        'color','r','Marker','v');
    drawnow;
    delete(prevPhiLine);
    delete(prevRotorNline);
    delete(prevRotorSline);
    prevPhiLine = phiLine;
    prevRotorNline = rotorPline;
    prevRotorSline = rotorNline;
%     pause(dT);
end
hold off;



function stopPlot(~,callbackdata,stopLoopCond)

switch callbackdata.Key
    case 'q'
        stopLoopCond.value = true;
    otherwise
        disp('Unknown command!');
end

end

