classdef FOCemulator < handle
    %FOCemulator Emulates 2FOC driver functions
    
    properties(Constant=true)
        % Sine, Cosine tables
        cos_table = [32767,32763,32748,32723,32688,32643,32588,32523,32449,32364,32270,32165,32051,31928,31794,31651,31498,31336,31164,30982,30791,30591,30381,30163,29935,29697,29451,29196,28932,28659,28377];
        sin_table = [    0,  330,  660,  990, 1319, 1648, 1977, 2305, 2632, 2959, 3285, 3609, 3933, 4255, 4576, 4896, 5214, 5531, 5846, 6159, 6470, 6779, 7087, 7392, 7694, 7995, 8293, 8588, 8881, 9171, 9459];
        clarkT = sqrt(2/3)*[
            1 cos(-2*pi/3) cos(-4*pi/3)
            0 sin(-2*pi/3) sin(-4*pi/3)
            1/sqrt(2) 1/sqrt(2) 1/sqrt(2)
            ];
    end
    
    properties
        enc@double;    % motor electric phase
        delta@double   % cosine sine argument
        sector@double; % electric phase sector
        negative_sec@logical;
        HI,LO,NE;      % virtual "rotating" terminal names
        cosT@double;
        sinT@double;
    end
    
    methods
        function obj = FOCemulator()
            obj.enc = 0;
            obj.sector = 0; % uninitialised
        end
        
        function updateState(obj,elecAngle)
            % update electric angle
            obj.enc = mod(elecAngle,360);
            
            % minimized angle (named "enc"  in the 2FOC code) from electric angle
            obj.delta = mod(obj.enc,60)-30;
            disp(obj.delta);
            
            % update sector
            obj.sector = floor(obj.enc/60)+1;
            obj.negative_sec = logical(mod(obj.sector,2));
            
            % update PWM pointers
            switch obj.sector
                case 1
                    [obj.HI,obj.LO,obj.NE] = deal('Ta','Tb','Tc');
                case 2
                    [obj.HI,obj.LO,obj.NE] = deal('Ta','Tc','Tb');
                case 3
                    [obj.HI,obj.LO,obj.NE] = deal('Tb','Tc','Ta');
                case 4
                    [obj.HI,obj.LO,obj.NE] = deal('Tb','Ta','Tc');
                case 5
                    [obj.HI,obj.LO,obj.NE] = deal('Tc','Ta','Tb');
                case 6
                    [obj.HI,obj.LO,obj.NE] = deal('Tc','Tb','Ta');
                otherwise
                    error(['Wrong sector (' obj.sector ')!!']);
            end
            
            % update cos, sin
            obj.cosT = FOCemulator.cos_table(1+abs(obj.delta));
            obj.sinT = sign(obj.delta)*FOCemulator.sin_table(1+abs(obj.delta));
        end
        
        function [Id,Iq] = computeIqId(obj,Ia,Ib,Ic)
            %generatePWM Generate the d-q Current timeseries
            
            % real terminal output currents on terminals Ta, Tb, Tc
            Iphase = struct('Ta',Ia,'Tb',Ib,'Tc',Ic);
            
            % Remap the phase currents to i0, iL, iH
            i0 = Iphase.(obj.NE);
            iL = Iphase.(obj.LO);
            iH = Iphase.(obj.HI);
            
            % Compute q-d currents
            if (obj.negative_sec)
                Iq = floor(((iH-iL) * obj.cosT - 3*i0    * obj.sinT)*sqrt(1/2)/2^15);
                Id = floor((    -i0 * obj.cosT - (iH-iL) * obj.sinT)*sqrt(3/2)/2^15);
            else
                Iq = floor(((iH-iL) * obj.cosT + 3*i0    * obj.sinT)*sqrt(1/2)/2^15);
                Id = floor((     i0 * obj.cosT - (iH-iL) * obj.sinT)*sqrt(3/2)/2^15);
            end
        end
        
        function [Va,Vb,Vc] = computePWMabc(obj,Vd,Vq)
            %generatePWM Generate the PWM timeseries on terminals Ta, Tb, Tc
            
            % real terminal output voltages on terminals Ta, Tb, Tc
            pwm = struct('Ta',0,'Tb',0,'Tc',0);
            % CtrlReferences.VqRef = Vq/100*32000;
            % VqRef = ((long)CtrlReferences.VqRef)<<(10-VOLT_REF_SHIFT)
            %       = CtrlReferences.VqRef*2^5
            % Vq = VqRef/2^10
            % => VqRef = Vq/100*32000/2^5
            Vd = Vd/100*32000/2^5; % Vd*10
            Vq = Vq/100*32000/2^5; % Vq*10
            
            % Compute outputs
            V1 = floor((Vq * obj.cosT - 3*Vd * obj.sinT)*sqrt(1/6)/2^15);
            V2 = floor((Vq * obj.sinT +   Vd * obj.cosT)*sqrt(1/6)/2^15);
            
            if (obj.negative_sec), V2=-V2; end
                
                % Map the values to the actual terminals
                pwm.(obj.HI) =  V1-V2;
                pwm.(obj.NE) =  V2+V2;
                pwm.(obj.LO) = -V1-V2;
            
            [Va,Vb,Vc] = deal(pwm.Ta,pwm.Tb,pwm.Tc);
        end
    end
end
