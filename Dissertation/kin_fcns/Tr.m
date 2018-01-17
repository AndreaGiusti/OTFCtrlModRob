% Tr - Perform translation using homogeneous transformation
%      matrix along an axis
%
% Syntax:  
%    Tr(axis,L)
%
% Inputs: 
%    axis -> desired axis for translation ('x', 'y' or 'z')
%    L    -> amount of translation (scalar)
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


function A = Tr(axis,L)

switch axis
    case 'x'
        A = [1, 0, 0, L;
             0, 1, 0, 0;
             0, 0, 1, 0;
             0, 0, 0, 1];
    case 'y'
        A = [1, 0, 0, 0;
             0, 1, 0, L;
             0, 0, 1, 0;
             0, 0, 0, 1];
    case 'z'
        A = [1, 0, 0, 0;
             0, 1, 0, 0;
             0, 0, 1, L;
             0, 0, 0, 1];
    otherwise
        error('Wrong selection of the translation axis.')
end

end