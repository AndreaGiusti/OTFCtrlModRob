% main_test - Script for testing functions
% Syntax:  
%
% Inputs: 
%
% Example: 
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: 

% Author:       Andrea Giusti
% Written:      01-12-2015
% Last update:  05-03-2017
% Last revision:---

%------------- BEGIN CODE --------------

%clc;
%close all;
%clear;

% Add folders to the path
addpath('dyn_fcns','kin_fcns','modulesDB');

% Initialize the DB of modules of the Schunk LWA4P arm
init_SchunkModDB;

%% Robot Assembly
ModRob = [PB1; L1; PB2; L2; PB3; L3];

%% Synthesis of kin. parameters
% get extended DH table and pose of the base frame from ModRob
[DHext,B]   = ModRob2DHext(ModRob);
% get standard DHtab from DHext
[DHtab, jt] = DHext2DH(DHext);

%% Synthesis of dyn. parameters
DynPar = ModRob2Dynpar(ModRob,DHext);

%% Call numerically the modified recursive N-E algorithm 

KinPar.B     = B(1:3,1:3);
KinPar.DHext = DHext;
KinPar.NJ    = get_NJ(ModRob);
gravity      = -9.81; 


q   = zeros(KinPar.NJ,1);
dq  = zeros(KinPar.NJ,1);
dqA = zeros(KinPar.NJ,1);
ddq = zeros(KinPar.NJ,1);

% compute inverse dynamics
tau = NE_mod(q,dq,dqA,ddq,KinPar,DynPar,gravity);

