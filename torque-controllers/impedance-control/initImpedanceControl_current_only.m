%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /**
%  * Copyright (C) 2016 CoDyCo
%  * @author: Daniele Pucci
%  * Permission is granted to copy, distribute, and/or modify this program
%  * under the terms of the GNU General Public License, version 2 or any
%  * later version published by the Free Software Foundation.
%  *
%  * A copy of the license can be found at
%  * http://www.robotcub.org/icub/license/gpl.txt
%  *
%  * This program is distributed in the hope that it will be useful, but
%  * WITHOUT ANY WARRANTY; without even the implied warranty of
%  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  * Public License for more details
%  */1
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear variables
clc

%% GENERAL SIMULATION INFO
% If you are simulating the robot with Gazebo, 
% remember that you have to launch Gazebo as follow:
% 
% gazebo -slibgazebo_yarp_clock.so
% 
% and set the environmental variable YARP_ROBOT_NAME = icubGazeboSim. 

setenv('YARP_ROBOT_NAME','iCubGenova04');
% setenv('YARP_ROBOT_NAME','iCubGenova02');
% setenv('YARP_ROBOT_NAME','icubGazeboSim');
% setenv('YARP_ROBOT_NAME','iCubGazeboV2_5');

% simulation time  
Config.Ts              = 0.001;
Config.SIMULATION_TIME = inf;  

% Default behaviour: gravity compensation. If MOVING = true, the robot will also move 
% all actuated joints following a sine trajectory
MOVING                = false;

% If Config.SAVE_WORKSPACE = True, every time the simulink model is run the
% workspace is saved after stopping the simulation
Config.SAVE_WORKSPACE = false;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONFIGURATIONS COMPLETED: loading gains and parameters for the specific robot
run(strcat('app/robots/',getenv('YARP_ROBOT_NAME'),'/configRobot_current_only.m')); 
