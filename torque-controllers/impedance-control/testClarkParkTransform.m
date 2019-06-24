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

% Rotation
matElem = zeros(1,1,length(pos));
[theta,vec0, vec1] = deal(matElem);
theta(1,1,:) = pos/180*pi*6;
vec0(1,1,:) = 0;
vec1(1,1,:) = 1;
R = [
    cos(theta) -sin(theta) vec0
    sin(theta)  cos(theta) vec0
    vec0        vec0       vec1
    ];

expIq = zeros(size(pos));
aLine = [];

for degShift = -10:10
    
    % Power conservative Clark transform
    phaseDelta = degShift*pi/180*6;
    ClarkT = 2*sqrt(1/3)*[
        1 cos(-2*pi/3+phaseDelta) cos(-4*pi/3+2*phaseDelta)
        0 sin(-2*pi/3+phaseDelta) sin(-4*pi/3+2*phaseDelta)
        1/sqrt(2) 1/sqrt(2) 1/sqrt(2)
        ];
    
    for idx = 1:length(pos)
        qIaqIbqIc = [i1(idx) i2(idx) i3(idx)]';
        expIdIqI0 = R(:,:,idx)*ClarkT*qIaqIbqIc;
        expIq(idx) = expIdIqI0(2);
    end
    oldLine = aLine;
    aLine=plot(pos,expIq);
    pause; display(degShift);
    if ~isempty(oldLine)
        delete(oldLine);
    end
    
end
