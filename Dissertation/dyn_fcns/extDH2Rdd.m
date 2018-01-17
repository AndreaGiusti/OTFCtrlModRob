% extDH2Rdd - Extracts the vector from PJim1 and PJi for each link i
%
% Syntax:  
%    extDH2Rdd(extDH_row,q)
%
% Inputs: 
%    q         -> i-th joint variable
%    extDH_row -> i-th raw of the extended DH table
%
% Outputs:
%   r_dd -> vector from PJim1 and PJi
%
% Example: 
%
% Other m-files required: Tr(), Ro()                    
% Subfunctions: none
% MAT-files required: none
%
% See also: 

% Author:       Andrea Giusti
% Written:      14-December-2015
% Last update:  ---
% Last revision:---

%------------- BEGIN CODE --------------

function r_dd = extDH2Rdd(extDH_row,q)

Aim1_i = eye(4);

% DH_ext; % each row is [a_i, alpha_i, gamma_i, d_i, p_i, n_i, jt, phi]
a_i     = extDH_row(1);
alpha_i = extDH_row(2);
gamma_i = extDH_row(3);
p_i     = extDH_row(5);
n_i     = extDH_row(6);
jt      = extDH_row(7);

if jt == 0
    Aim1_i = Tr('z',-p_i)*Ro('z',gamma_i+q)*Tr('x',a_i)*Ro('x',alpha_i)*Tr('z',n_i);
elseif jt == 1
    Aim1_i = Tr('z',-p_i+q)*Ro('z',gamma_i)*Tr('x',a_i)*Ro('x',alpha_i)*Tr('z',n_i);
end
Ai_im1 = inv(Aim1_i);
r_dd = -Ai_im1(1:3,4);
end