function [focPosUnits,focVelUnits] = motorYarpPos2motorfocPosUnits(yarpPosUnits)
%motorYarpPos2motorfocPosUnits Converts motor yarp position units to motor fullscaleticks (2FOC)
%   Returns also the motor velocity 2FOC units.
%   
%   motor position units in 2FOC (mpu2foc): fullscaleticks
%   motor position units in yarp (mpuyarp): rad
%   A full rotor turn has 14400 ticks (LCORE encoder resolution).
%   In the 2FOC firmware code, the "gQEPosition" variable is actually computed
%   from POSCNT (ticks) as follows:
%   mpu2foc = POSCNT<<16 / QE_RESOLUTION = POSCNT*65536/14400 fullscaleticks
%   
%   (mpu2foc*14400/65536) --> ticks
%   * 2pi/14400  --> rad
%   so, mpuyarp = (mvu2foc*14400/65536) * (2*pi/14400)
%               = mvu2foc*2*pi/65536.
%   
%   In the 2FOC firmware code, the "gQEVelocity" variable is actually computed
%   as a delta of fullscaleticks per millisecond:
%   
%   mpuyarp (rad/ms) * 1000 --> rad/s

persistent fullscaleticks;
fullscaleticks = hex2dec('10000');

focPosUnits = yarpPosUnits*fullscaleticks/(2*pi);
focVelUnits = focPosUnits/1000;

end

