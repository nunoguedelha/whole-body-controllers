% Gains and parameters for impedance controller
Config.ON_GAZEBO = false;
% ROBOT_DOF        = 23;
ROBOT_DOF        = 1;

% Robot configuration for WBT3.0
WBTConfigRobot           = WBToolbox.Configuration;
WBTConfigRobot.RobotName = 'icub';
WBTConfigRobot.UrdfFile  = 'model.urdf';
WBTConfigRobot.LocalName = 'WBT';

% WBTConfigRobot.ControlBoardsNames = {'torso','left_arm','right_arm','left_leg','right_leg'};
% WBTConfigRobot.ControlledJoints   = {'torso_pitch','torso_roll','torso_yaw', ...
%                                      'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow', ...
%                                      'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow', ...
%                                      'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll', ...
%                                      'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};
WBTConfigRobot.ControlBoardsNames = {'right_leg'};
WBTConfigRobot.ControlledJoints   = {'r_knee'};
% WBTConfigRobot.ControlledJoints   = {'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};

% Gains of the torque-to-current control
Kc = 0.0;
Kv = 0.000655;
Kt = 0.187259;
% Kc = [ 0.0 0.0 0.0 0.0 0.0 0.0 ];
% Kv = [ 0.001013 0.000458 0.000980 0.000655 0.001343 0.000520 ];
% Kt = [ 0.142674 -0.158045 0.132443 0.187259 -0.150921 -0.096339 ];

% References for the demo movements
if MOVING
    
    % Impedance gains
    Kp     = 44*diag(ones(1,ROBOT_DOF));
    Kd     = 3.0*sqrt(Kp);
    
    AMPLS  = 6.0*ones(1,ROBOT_DOF);
    FREQS  = 0.5*ones(1,ROBOT_DOF);
   
else
    
    % Impedance gains
    Kp     = 0*diag(ones(1,ROBOT_DOF));
    Kd     = 0*diag(ones(1,ROBOT_DOF));
    
    AMPLS  = 0*ones(1,ROBOT_DOF);
    FREQS  = 0*ones(1,ROBOT_DOF);
end
    
if size(Kp,1) ~= ROBOT_DOF
    error('Dimension of Kp different from ROBOT_DOF')
end
