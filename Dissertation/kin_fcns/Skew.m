% Skew - Skew symmetric operator
%
% Syntax:  
%    Skew(w)
%
% Inputs: 
%    w -> 3x1 vector
% Outputs:
%    S -> 3x3 skew symmetric matrix
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

function S = Skew(w)
%SKEW  matrix
  if 3 ~= size(w,1),
    error('SCREWS:skew','vector must be 3x1')
  end
  
  if isnumeric(w),
    S = zeros(3,3);
  end
  
  S(1,2) = -w(3);
  S(1,3) =  w(2);
  S(2,3) = -w(1);

  S(2,1) =  w(3);
  S(3,1) = -w(2);
  S(3,2) =  w(1);
  
end