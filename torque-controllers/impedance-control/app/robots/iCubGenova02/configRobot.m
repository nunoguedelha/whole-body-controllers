% Gains and parameters for impedance controller
Config.ON_GAZEBO = false;

% Robot configuration for WBT3.0
WBTConfigRobot           = WBToolbox.Configuration;
WBTConfigRobot.RobotName = 'icub';
WBTConfigRobot.UrdfFile  = 'model.urdf';
WBTConfigRobot.LocalName = 'WBT';

% All available boards and joints
AllBoardsNames = {'torso','left_arm','right_arm','left_leg','right_leg'};
AllJoints = {'torso_yaw','torso_roll','torso_pitch', ...
             'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow', ...
             'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow', ...
             'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll', ...
             'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};
         
ControlledBoardNames = {'torso', 'left_arm', 'right_arm', 'left_leg', 'right_leg'}; % used when CONTROL_BOARD_1_OR_JOINTS_0_SELECTOR==1
ControlledJoints = {}; % used when CONTROL_BOARD_1_OR_JOINTS_0_SELECTOR==0

% configuration
CONTROL_BOARD_1_OR_JOINTS_0_SELECTOR = 1;
JTCVC_1_OR_MCC_0_SELECTOR = 1;
JOINTS_TO_PLOT = [ 1 ]; % joint for which current tau and qj is plotted (desired and measured)
JOINTS_TO_MOVE = [ 1 ]; % joints that are moved when MOVING==1

% motor to joint velocity transform
T_torso =  [0.55 0.275 0.275; 0.0 0.5 0.5; 0.0 -0.5 0.5];
T_r_arm = [1 0.0 0.0 0.0; 1.0 0.625 0.0 0.0; 0.0 -0.625 0.625 0.0; 0.0 0.0 0.0 1.0];
T_l_arm = [-1 0.0 0.0 0.0; -1.0 -0.625 0.0 0.0; 0.0 0.625 -0.625 0.0; 0.0 0.0 0.0 1.0];
T_r_leg = diag(ones(6,1));
T_l_leg = diag(ones(6,1));

% joint torque control via current gains
Kt_torso = [   -0.245227       -0.297433       -0.465476    ];
Kt_r_arm = [   -0.160695       -0.391429       -0.636335       -0.15    ];
Kt_l_arm = [    0.160695        0.391429        0.636335        0.15    ];
Kt_r_leg = [  -0.142674        0.158045      -0.132443      -0.187259        0.150921        0.096339    ];
Kt_l_leg = [   0.142674       -0.158045       0.132443       0.187259       -0.150921       -0.096339    ]; 

Kv_torso = [   0.000612       0.000436       0.000447    ];
Kv_r_arm = [   0.000249       0.000391       0.000361       0.0004    ];
Kv_l_arm = [   0.000249       0.000391       0.000361       0.0004    ];
Kv_r_leg = [   0.000457       0.000458       0.000980       0.000655       0.001343       0.000520    ];
Kv_l_leg = [   0.000457       0.000458       0.000980       0.000655       0.001343       0.000520    ];

Kc_torso = [ 0.0 0.0 0.0 ];
Kc_r_arm = [ 0.0 0.0 0.0 0.0 ];
Kc_l_arm = [ 0.0 0.0 0.0 0.0 ];
Kc_r_leg = [ 0.0 0.0 0.0 0.0 0.0 0.0 ];
Kc_l_leg = [ 0.0 0.0 0.0 0.0 0.0 0.0 ];

Kfc = 0.0;

% map board to joints
boardMap = {1:3, 4:7, 8:11, 12:17, 18:23};

% select joints, gains and transform matrix to control
if CONTROL_BOARD_1_OR_JOINTS_0_SELECTOR
    indexControlledBoardNames = zeros(length(ControlledBoardNames),1);
    for i=1:length(ControlledBoardNames)
        idx = find(strcmp(AllBoardsNames,ControlledBoardNames{i}));
        indexControlledBoardNames(i) = idx;
    end
    
    T_list = {T_torso', T_l_arm', T_r_arm', T_l_leg', T_r_leg'};
    T = blkdiag(T_list{indexControlledBoardNames});
    
    Kt_list = {Kt_torso, Kt_l_arm, Kt_r_arm, Kt_l_leg, Kt_r_leg};
    Kt = [Kt_list{indexControlledBoardNames}];
    Kv_list = {Kv_torso, Kv_l_arm, Kv_r_arm, Kv_l_leg, Kv_r_leg};
    Kv = [Kv_list{indexControlledBoardNames}];
    Kc_list = {Kc_torso, Kc_l_arm, Kc_r_arm, Kc_l_leg, Kc_r_leg};
    Kc = [Kc_list{indexControlledBoardNames}];
    
    WBTConfigRobot.ControlBoardsNames = ControlledBoardNames;
    WBTConfigRobot.ControlledJoints = AllJoints(cell2mat(boardMap(indexControlledBoardNames)));
else
    % TODO
end

% definition of robot number of DoF, and prepare selector vectors
ROBOT_DOF = length(WBTConfigRobot.ControlledJoints);

PLOT_SELECTOR = zeros(ROBOT_DOF, 1);
PLOT_SELECTOR(JOINTS_TO_PLOT) = 1;

MOVE_SELECTOR = zeros(1, ROBOT_DOF);
MOVE_SELECTORT(JOINTS_TO_MOVE) = 1;

% References for the demo movements
if MOVING
    
    % Impedance gains
    Kp     = 50*diag(ones(1,ROBOT_DOF));
    Kd     = 2.5*sqrt(Kp);
    
    AMPLS  = 10.0*MOVE_SELECTOR;
    FREQS  = 0.2*MOVE_SELECTOR;
    
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
if size(Kd,1) ~= ROBOT_DOF 
    error('Dimension of Kd different from ROBOT_DOF')
end
