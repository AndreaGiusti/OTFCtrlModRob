% DHext2DH -  Extracts the DH table from DHext
%
% Syntax:  
%    DHext2DH(DHext)
%
% Inputs: 
%    DHext -> Extended DH table, the i-th row is 
%             [a_i, alpha_i, gamma_i, d_i, p_i, n_i, jt_i, phi_i]
%             (refer to [1] for further details on the parameters).
%
% Outputs:
%    DHtab -> Standard DH table, the i-th row of DHtab is
%                  [a_i, alpha_i, gamma_i, d_i].
%    jt    -> Id vector of joint types (0 for rev., 1 for prism.)
%
% Example: 
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: 
% [1] A. Giusti and M. Althoff, "Automatic Centralized Controller Design for
% Modular and Reconfigurable Robot Manipulators", IROS 2015.

% Author:       Andrea Giusti
% Written:      14-December-2015
% Last update:  ---
% Last revision:---

%------------- BEGIN CODE --------------

function [DHtab, jt] = DHext2DH(DHext)
DHtab = DHext(:,1:4);
jt    = DHext(:,7); 
end
