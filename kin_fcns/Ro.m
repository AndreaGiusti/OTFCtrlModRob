% Ro - Perform an elementary rotation using homogeneous transformation
%      matrix around an axis
%
% Syntax:  
%    Ro(axis,L)
%
% Inputs: 
%    axis -> desired axis for translation ('x', 'y' or 'z')
%    angle -> angle of rotation (scalar)
% Outputs:
%    A -> 4x4 homogeneous transformation matrix
%
% Example: 
%
% Other m-files required: none       
% Subfunctions: none
% MAT-files required: none
%
% See also: 

% Author:       Andrea Giusti
% Written:      14-December-2015
% Last update:  ---
% Last revision:---

%------------- BEGIN CODE --------------

function A = Ro(axis,angle)
% Perform an elementary rotation using homogeneous transformation matrix
% around an axis
% Output: A     -> 4x4 homogeneous transformation matrix
%
% Input:  axis  -> desired axis of rotation ('x', 'y' or 'z') 
%         angle -> angle of rotation (scalar)

switch axis
    case 'x'
        A = [1, 0, 0, 0;
            0, cos(angle), -sin(angle), 0;
            0, sin(angle), cos(angle) , 0;
            0, 0, 0, 1];
    case 'y'       
        A = [cos(angle), 0, sin(angle), 0;
            0, 1, 0, 0;
            -sin(angle), 0, cos(angle), 0;
            0, 0, 0, 1];
    case 'z'
        A = [cos(angle), -sin(angle), 0, 0;
            sin(angle), cos(angle), 0, 0;
            0, 0, 1, 0;
            0, 0, 0, 1];
    otherwise
        error('Wrong selection of the rotation axis.')
end

end