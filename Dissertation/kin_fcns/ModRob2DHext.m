% ModRob2DHext -  Returns the extended D-H table using information in ModRob
%
% Syntax:  
%    ModRob2DHext(ModRob)
%
% Inputs: 
%    ModRob -> Ordered array of the module structures with respect to 
%              the robot assembly 
%
% Outputs:
%    DHext_out -> Extended DH table of parameters, the i-th row is 
%                 [a_i, alpha_i, gamma_i, d_i, p_i, n_i, jt_i, phi_i],
%                 (refer to [1] for further details on the parameters).
%    B         -> B is the hom. transf. matrix from the basis to the 
%                 first joint
%
% Example: 
%
% Other m-files required: Tr(axis,L), Ro(axis,angle)                      
% Subfunctions: compute_ModKinInfov2(F,phi_im1,delJ_im1,guard)
% MAT-files required: none
%
% See also: 
% [1] A. Giusti and M. Althoff, "Automatic Centralized Controller Design for
% Modular and Reconfigurable Robot Manipulators", IROS 2015.

% Author:       Andrea Giusti
% Written:      14-December-2015
% Last update:  Andrea Giusti 05.03.2017
% Last revision:---

%------------- BEGIN CODE --------------

function [DHext_out,B] = ModRob2DHext(ModRob)
% Initialize number of modules that compose the assembly and number of
% joints
N =  0;
NJ = 0;
for k =1:length(ModRob)
    if ModRob(k).Mod.ID ~= 0
        N = N+1;
    end
    if (ModRob(k).Mod.ID ~= 0 && ModRob(k).Mod.typ == 1)
        NJ = NJ+1;
    end
end

n_pl_i = 0;

% initialize output variables
phi      = zeros(length(ModRob),1);
kin_info = zeros(length(ModRob),5);
DHext    = zeros(length(ModRob),8);

% Compute the static transformation until the first joint
% check whether the first module is a link module
h = 1;
B = eye(4);
while(ModRob(h).Mod.typ == 2 && h <= N)
    p_l         = ModRob(h).Kpl.p_pl;
    a_l         = ModRob(h).Kpl.a_pl;
    n_l         = ModRob(h).Kpl.n_pl;
    alpha_l     = ModRob(h).Kpl.alpha_pl;
    delta_l_in  = ModRob(h).Kpl.delta_pl;
    delta_l_out = ModRob(h).Kdl.delta_dl;
    B = B*Tr('z',-p_l)*Ro('z',-delta_l_in)*Tr('x',a_l)...
         *Ro('x',alpha_l)*Tr('z',n_l)*Ro('z',delta_l_out);
    h=h+1;
end
p_pl_0     = ModRob(h).Kpl.p_pl;
a_pl_0     = ModRob(h).Kpl.a_pl;
n_pl_0     = ModRob(h).Kpl.n_pl;
alpha_pl_0 = ModRob(h).Kpl.alpha_pl;
delta_pl_0 = ModRob(h).Kpl.delta_pl;
B = B*Tr('z',-p_pl_0)*Ro('z',-delta_pl_0)*Tr('x',a_pl_0)...
    *Ro('x',alpha_pl_0)*Tr('z',n_pl_0);

% from the first joint module found start core procedure
k = h;
cntJ = 0;
while k <= (N-1)
% A joint module
cntJ = cntJ+1;

p_dl_im1     = ModRob(k).Kdl.p_dl;
a_dl_im1     = ModRob(k).Kdl.a_dl;
n_dl_im1     = ModRob(k).Kdl.n_dl;
alpha_dl_im1 = ModRob(k).Kdl.alpha_dl;
delta_dl_im1 = ModRob(k).Kdl.delta_dl;
deltaJ_im1   = ModRob(k).Kj.delta_j;
JT_im1       = ModRob(k).Kj.jt;

% check if the next module is a link module
f = k+1;
H = eye(4);

    while ModRob(f).Mod.typ == 2
        p_l         = ModRob(f).Kpl.p_pl;
        a_l         = ModRob(f).Kpl.a_pl;
        n_l         = ModRob(f).Kpl.n_pl;
        alpha_l     = ModRob(f).Kpl.alpha_pl;
        delta_l_in  = ModRob(f).Kpl.delta_pl;
        delta_l_out = ModRob(f).Kdl.delta_dl;
        H = H*Tr('z',-p_l)*Ro('z',-delta_l_in)*Tr('x',a_l)...
            *Ro('x',alpha_l)*Tr('z',n_l)*Ro('z',delta_l_out);
        f=f+1;
    end
    k = f-1;


p_pl_i     = ModRob(f).Kpl.p_pl;
a_pl_i     = ModRob(f).Kpl.a_pl;
n_pl_i     = ModRob(f).Kpl.n_pl;
alpha_pl_i = ModRob(f).Kpl.alpha_pl;
delta_pl_i = ModRob(f).Kpl.delta_pl;

F_aux = Tr('z',-p_dl_im1)*Tr('x',a_dl_im1)*Ro('x',alpha_dl_im1)*...
        Tr('z',n_dl_im1)*Ro('z',delta_dl_im1)*H*Ro('z',-delta_pl_i)*...
        Tr('z',-p_pl_i)*Tr('x',a_pl_i)*Ro('x',alpha_pl_i)*Tr('z',n_pl_i);

zim1 = [0 0 1]';
zi = F_aux(1:3,3);
pxyz_im1 = F_aux(1:3,4);
Rim1_i = F_aux(1:3,1:3);
    % to avoid numerical problems we introduce a guard
    guard = 1e-3;
    % init impact that a delta has on the next module
    phi(cntJ) = 0;
    % Case 1: zim1 and zi are overlapped
    if (norm(cross(zi,zim1)) < guard) && (norm(pxyz_im1(1:2)) < guard)
        phi(cntJ) = 0;
        % Case 2: zim1 and zi are parallel
    elseif norm(cross(zi,zim1)) < guard
        % Be careful, the angle is expressed in frame im1 but the
        % rotation must be performed in frame i, so that the
        % angle in i must be determined.
        pxyz_i = transpose(Rim1_i)*pxyz_im1;
        phi(cntJ) = atan2(pxyz_i(2),pxyz_i(1));
        % Case 3: zim1 and zi are skew or intersect
    else
        nx_im1 = abs(cross(zi,zim1));
        nx_i = transpose(Rim1_i)*nx_im1;
        phi(cntJ) = atan2(nx_i(2),nx_i(1));
    end
    % The x axis of the frame of reference i must be aligned with
    % the DH one.
    F = F_aux*Ro('z', phi(cntJ));
    if cntJ > 1
        kin_info(cntJ,:) = compute_ModKinInfov2(F,phi(cntJ-1),deltaJ_im1,guard); %kin_info = [a_0 alpha_0 gamma_0 p_0 n_0];
        DHext(cntJ,1:7) = [kin_info(cntJ,1) kin_info(cntJ,2) kin_info(cntJ,3) kin_info(cntJ-1,5)-kin_info(cntJ,4) kin_info(cntJ,4) kin_info(cntJ,5) JT_im1];
    else
        kin_info(cntJ,:) = compute_ModKinInfov2(F,0,deltaJ_im1,guard);
        DHext(cntJ,1:7) = [kin_info(cntJ,1) kin_info(cntJ,2) kin_info(cntJ,3) -kin_info(cntJ,4) kin_info(cntJ,4) kin_info(cntJ,5) JT_im1];
    end
    
    k=k+1;
end

cntJ = cntJ+1;
% The last link is composed of the distal part only!
if N >0
    p_dl_N     = ModRob(N).Kdl.p_dl;
    n_dl_N     = ModRob(N).Kdl.n_dl;
    a_dl_N     = ModRob(N).Kdl.a_dl;
    alpha_dl_N = ModRob(N).Kdl.alpha_dl;
    delta_J_N  = ModRob(N).Kj.delta_j;
    JT_N       = ModRob(N).Kj.jt;
    DHext(cntJ,1:7) = [a_dl_N alpha_dl_N delta_J_N-phi(cntJ-1) DHext(cntJ-1,6)-p_dl_N p_dl_N n_dl_N JT_N]; 
    DHext(1:NJ,8) = [0; phi(1:NJ-1,1)];%[0;phi'];
end
DHext_out = DHext;

end
