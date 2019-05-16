% Gains and parameters for impedance controller
Config.ON_GAZEBO = false;
% ROBOT_DOF        = 23;
ROBOT_DOF        = 1;

% constants
OUTPUT_PWM = 1;
OUTPUT_CURRENT = 2;

% Robot configurations for WBT3.0
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
         
ControlledBoardNames = {'left_leg'}; % used when CONTROL_BOARD_1_OR_JOINTS_0_SELECTOR==1
ControlledJoints = {}; % used when CONTROL_BOARD_1_OR_JOINTS_0_SELECTOR==0

% configuration
JOINTS_TO_PLOT = [ 2 ]; % joint for which current tau and qj is plotted (desired and measured)
JOINTS_TO_MOVE = [ 2 ]; % joints that are moved when MOVING==1
CONTROLLER_OUTPUT = OUTPUT_CURRENT;

% map board to joints
boardMap = {1:3, 4:7, 8:11, 12:17, 18:23};
Variant_1 = Simulink.Variant; Variant_1.Condition = 'CONTROLLER_OUTPUT == OUTPUT_PWM';
Variant_2 = Simulink.Variant; Variant_2.Condition = 'CONTROLLER_OUTPUT == OUTPUT_CURRENT';

indexControlledBoardNames = zeros(length(ControlledBoardNames),1);
for i=1:length(ControlledBoardNames)
    idx = find(strcmp(AllBoardsNames,ControlledBoardNames{i}));
    indexControlledBoardNames(i) = idx;
end

WBTConfigRobot.ControlBoardsNames = ControlledBoardNames;
WBTConfigRobot.ControlledJoints = AllJoints(cell2mat(boardMap(indexControlledBoardNames)));

% definition of robot number of DoF, and prepare selector vectors
ROBOT_DOF = length(WBTConfigRobot.ControlledJoints);

PLOT_SELECTOR = zeros(ROBOT_DOF, 1);
PLOT_SELECTOR(JOINTS_TO_PLOT) = 1;

MOVE_SELECTOR = zeros(1, ROBOT_DOF);
MOVE_SELECTOR(JOINTS_TO_MOVE) = 1;

AMPLS_SETPOINT_SCALAR = 2;
FREQS_SETPOINT_SCALAR = 0.1;

AMPLS_SETPOINT = AMPLS_SETPOINT_SCALAR*MOVE_SELECTOR;
FREQS_SETPOINT = FREQS_SETPOINT_SCALAR*MOVE_SELECTOR;

CONST_SETPOINT = 3*MOVE_SELECTOR;
