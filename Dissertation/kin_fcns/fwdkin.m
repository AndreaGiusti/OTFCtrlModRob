% fwdkin - forward/direct kinematics
%
% Syntax:  
%    [p,R] = fwdkin(q,DHtab,jt);
%
% Inputs: 
%    q      (n x 1 matrix)     -> joint position
%    DHtab                     -> Standard DH table, the i-th row of DHtab is
%                                       [a_i, alpha_i, gamma_i, d_i].
%    jt                        -> Id vector of joint types (0 for rev., 1 for prism.)
%
% Outputs:
%    p    (3 x 1 matrix)       -> Cartesian position in Base-Frame 
%    R    (3 x 3 matrix)       -> Rotation from TCP-Frame into Base-Frame
%
% Example: 
%
%  
% Other m-files required:   none
% Subfunctions:             dh_trans(a,alpha,theta,d)
% MAT-files required:       none
%
%
% Author:       Stefan Liu
% Written:      12-January-2015
% Last update:  25-January-2016
% Last revision:---

%------------- BEGIN CODE --------------
function [p,R] = fwdkin(q,DHtab,jt)
%#codegen
[N, ~]= size(DHtab);
T_a = eye(4);
for i=1:N
    T = dh_trans(DHtab(i,1),DHtab(i,2),DHtab(i,3) + not(jt(i))*q(i),DHtab(i,4) + jt(i)*q(i));
    T_a = T_a*T;
end
R = T_a(1:3,1:3);
p = T_a(1:3,4);
end

%% Subfunction: DH-Transformation Matrix
function T = dh_trans(a,alpha,theta,d)
T =[cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);...
    sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);...
    0,           sin(alpha),             cos(alpha),            d;...
    0,           0,                      0,                     1];
end
