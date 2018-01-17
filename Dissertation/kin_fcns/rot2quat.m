% rot2quat - Calculates unit quaternions of a given rotation matrix
%
% Syntax:  
%    [eta,epsilon] = rot2quat(R)
%
% Inputs: 
%    R -> 3x3 rotation matrix
% Outputs:
%    eta -> scalar
%    epsilon -> 3x1 vector
%
% Example: 
%
% Other m-files required: none       
% Subfunctions: none
% MAT-files required: none
%
% See also: 
% [1] S. Chiaverini and B. Siciliano, "The Unit Quaternion: A Useful Tool for 
% Inverse Kinematics of Robot Manipulators" in "Systemw Analysis, Modelling and Simulation", 1999.

% Author:       Stefan Liu
% Written:      18-January-2016
% Last update:  25-January-2016
% Last revision:---

%------------- BEGIN CODE --------------
function [eta, epsilon] = rot2quat(R)
%#codegen
% Calculate unit quaternion epsilon from 3x3 Rotation Matrix
epsilon = zeros(3,1);
epsilon(1) = 0.5*sign(R(3,2)-R(2,3)) * real(sqrt(complex(R(1,1)-R(2,2)-R(3,3) + 1)));
epsilon(2) = 0.5*sign(R(1,3)-R(3,1)) * real(sqrt(complex(R(2,2)-R(3,3)-R(1,1) + 1)));
epsilon(3) = 0.5*sign(R(2,1)-R(1,2)) * real(sqrt(complex(R(3,3)-R(1,1)-R(2,2) + 1)));
% Calculate unit quaternion eta from 3x3 Rotation Matrix
eta = 0.5 * real(sqrt(complex(R(1,1) + R(2,2) + R(3,3) + 1)));
end