% compute_ModKinInfov2 -  Computes the extended kin. pars. from a 
% cumulative transformation  of a i-th link F_i
%
% Syntax:  
%   compute_ModKinInfov2(F,phi_im1,delJ_im1,guard)
%
% Inputs: 
%    F        -> Synthesis matrix, see (3) of [1]
%    phi_im1  -> additional rotation for DH alignement (see (3) of [1])
%    delJ_im1 -> angular joint offset
%    guard    -> guard to avoid numerical issues when joint axes 
%                are parallel 
% Outputs:
%    kin_info -> [a_i alpha_i gamma_i p_i n_i] see pag. 5 of [1]
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

function kin_info = compute_ModKinInfov2(F,phi_im1,delJ_im1,guard)

N = F(1:3,1);
O = F(1:3,2);
A = F(1:3,3);
P = F(1:3,4);

alpha_0 = atan2(O(3),A(3));
gamma_0 = atan2(N(2),N(1))+delJ_im1-phi_im1;
a_0 = P(1)*N(1)+P(2)*N(2);
if abs(O(3))< guard
    n_0 = 0;
    p_0 = -P(3);
else
    n_0 = (P(1)*N(2)-P(2)*N(1))/O(3);
    p_0 = (P(1)*A(3)*N(2)-P(3)*O(3)-P(2)*A(3)*N(1))/O(3);
end

kin_info = [a_0 alpha_0 gamma_0 p_0 n_0];
end