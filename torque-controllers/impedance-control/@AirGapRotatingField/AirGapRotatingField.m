classdef AirGapRotatingField < handle
    %AirGapRotatingField Emulates an air-gap rotating field
    %   Detailed explanation goes here
    
    properties(Access=private)
        p@double; % number of paires of poles
        % phase inductances
        Lu@double;
        Lv@double;
        Lw@double;
        % the location where we measure the magnetic flux in the air-gap is defined as an 
        % angular position
        thetaPhi;
        % Flux vector field generated by phase U, V, W
        i2PHI_uvw;
        % Reading axis angular elec positions on U, V, W
        readingAxisPos_u = 0;
        readingAxisPos_v = 2*pi/3;
        readingAxisPos_w = 4*pi/3;
        readingAxesPosSeq@struct;
    end
    
    methods
        function obj = AirGapRotatingField(p,Lu,Lv,Lw,thetaPhi)
            [obj.p,obj.Lu,obj.Lv,obj.Lw,obj.thetaPhi] = deal(p,Lu,Lv,Lw,thetaPhi);
            
            % The magnetic field B is prop to the auxiliary magnetic field H:
            % B = mu*H
            % B is the magnetix flux density, and the magnetix flux phi = L*i.
            % Distribution of the magnetix flux generated by a single phase
            i2PHI_ph = @(L_ph,thetaPhi,readingAxisPos) L_ph*cos(p*thetaPhi-readingAxisPos);
            
            % Compute the Flux vector field generated by phase U, V, W
            obj.i2PHI_uvw = [
                i2PHI_ph(Lu,obj.thetaPhi,obj.readingAxisPos_u)
                i2PHI_ph(Lv,obj.thetaPhi,obj.readingAxisPos_v)
                i2PHI_ph(Lw,obj.thetaPhi,obj.readingAxisPos_w)];
            
            % Set reading axes distribution
            seq = repmat(...
                {'u','v','w'}',...
                [1 p+1]); % each column maps an electrical cycle
            positions = repmat(...
                [obj.readingAxisPos_u,obj.readingAxisPos_v,obj.readingAxisPos_w]',...
                [1 p+1]);
            offsets = repmat(0:(2*pi):(p*2*pi),[3 1]);
            pos = num2cell(positions+offsets);
            obj.readingAxesPosSeq = struct(...
                'seq',seq(:),...
                'pos',pos(:));
        end
        
        function paramValue = get(obj,parameterName)
            paramValue = obj.(parameterName);
        end
        
        function PHI_uvw = computePHI_uvw(obj,i_uvw)
            PHI_uvw = i_uvw(:)'*obj.i2PHI_uvw;
        end
    end
end
