% dJacobian - Calculation of the derivative of the geometric Jacobian 
%             given DH-Table
%
% Syntax:  
%    dJ = dJacobian(q,dq,DHtab,type)
%
% Inputs: 
%    q     -> Current Joint position vector (n x 1 matrix)
%    dq    -> Current Joint velocity vector (n x 1 matrix)
%    DHtab -> Standard DH table, the i-th row of DHtab is
%            [a_i, alpha_i, gamma_i, d_i].
%    type  -> (optional) Id vector of joint types (0 for rev., 1 for prism.)
%
% Outputs:
%    dJ     -> derivative of the Geometric Jacobian (6 x n Matrix)
%    J      -> Geometric Jacobian (6 x n Matrix)
% Example: 
%
% Other m-files required: none
% Subfunctions: dh_trans
% MAT-files required: none
%
% See also: 
% 

% Author:       Andrea Giusti
% Written:      23-April-2017
% Last update:  ---
% Last revision:---

%------------- BEGIN CODE --------------
function [dJ,J] = dJacobian(q,dq,DHtab,jt)
%#codegen

% Initialization
[N, ~]= size(DHtab);
prismatic = 1;
revolute = 0;

if nargin < 3 %set default value for type
    jt = revolute*ones(N,1);
end

%Define Input
alpha = DHtab(:,2);
a = DHtab(:,1);
d = DHtab(:,4) + jt.*q; 
theta = DHtab(:,3) + not(jt).*q;

%Define Output and auxiliary variables
if isnumeric(DHtab)%Numeric Calculation
    J  = zeros(6,N);
    dJ = zeros(6,N);
    z  = zeros(3,N+1);
    p  = zeros(3,N+1);
    dp = zeros(3,N+1);
    omega = zeros(3,N+1);
else %Symbolic Calculation
    J  = sym(zeros(6,N));
    dJ = sym(zeros(6,N));
    z  = sym(zeros(3,N+1));
    p  = sym(zeros(3,N+1));
    dp = sym(zeros(3,N+1));
    omega = sym(zeros(3,N+1));
end
T_a = eye(4);
z(:,1) = [0;0;1]; % this is z_0, convention: z_k = z(:,k+1)
p(:,1) = [0;0;0]; % this is p_0, convention: p_k = p(:,k+1)
p_tilde_0 = [0;0;0;1]; % tilde denotes the omogeneous vectors

% Calculate Position Vectors p_e, p and z
for i=1:N
    T = dh_trans(a(i),alpha(i),theta(i),d(i));
    T_a = T_a*T;
    p_tilde = T_a*p_tilde_0;
    p(:,i+1) = p_tilde(1:3);
    z(:,i+1) = T_a(1:3,1:3)*z(:,1);
end
p_e = p(:,N+1);

% Fill-in the Jacobian J and compute the velocities
for i=1:N
    if jt(i) == prismatic
        J(1:3,i) = z(:,i);
        J(4:6,i) = [0;0;0];
        
        omega(:,i+1) = omega(:,i);
        dp(:,i+1)    = cross(omega(:,i+1),p(:,i+1)-p(:,i))+dp(:,i)+z(:,i)*dq(i);
    else%jt(i) == revolute
        J(1:3,i) = cross(z(:,i),p_e - p(:,i));
        J(4:6,i) = z(:,i);
        
        omega(:,i+1) = z(:,i)*dq(i)+omega(:,i);
        dp(:,i+1)    = cross(omega(:,i+1),p(:,i+1)-p(:,i))+dp(:,i);
    end   
   % omega(:,i+1) = J(4:6,i)*dq(i)+omega(:,i);
   % dp(:,i+1)    = J(1:3,i)*dq(i)+dp(:,i);  
end
dp_e = J(1:3,:)*dq;


% Fill-in the derivative of the Jacobian dJ
for i=1:N
    if jt(i) == prismatic
        dJ(1:3,i) = cross(omega(:,i),z(:,i));
        dJ(4:6,i) = [0;0;0];
    else%jt(i) == revolute
            dJ(1:3,i) = cross(cross(omega(:,i),z(:,i)),p_e - p(:,i))+ ...
                cross(z(:,i),dp_e - dp(:,i));
            dJ(4:6,i) = cross(omega(:,i),z(:,i));
    end
end

end

%% DH-Helper
% D - homogeneous transformation matrix of a system in i into a system in i-1
function D = dh_trans(a, alpha,theta, d)

D=[cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);...
    sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);...
    0, sin(alpha), cos(alpha), d;...
    0,0,0,1];

end
