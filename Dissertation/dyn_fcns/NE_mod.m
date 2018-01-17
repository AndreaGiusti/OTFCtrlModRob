% NE_mod - Performs the modified NE algorithm of [1] and its small
%          extension as in [2] for prismatic joints
%        
%
% Syntax:  
%   NE_mod(q,dq,dqA,dqd,ddq,KinPar,DynPar,g)
%
% Inputs: 
%    q      -> vector of generalized joint positions
%    dq     -> vector of generalized joint velocities
%    dqA    -> vector of auxiliary generalized joint velocities
%    dqd    -> vector of desired joint velocities
%    ddq    -> vector of generalized joint accelerations
%    KinPar -> structure containing kin pars: KinPar.B, KinPar.DHext, 
%              KinPar.NJ
%    DynPar -> array of structures collecting dyn. pars of each link of the
%              assembled robot
%    g      -> gravitional acceleration
%
% Outputs:
%    tao_out -> vector of generalized forces/torques at the joints
%               (length(DynPar,1))
%
% Other m-files required: extDH2Rdd(extDH_row,q)
%                                             
% MAT-files required: none
%
% See also: 
% [1] A. De Luca, L. Ferrajoli, "A modified newton-euler method for 
% dynamic computations in robot fault detection and control", ICRA 2009.
% [2] A. Giusti and M. Althoff, "Efficient computation of 
% interval-arithmetic-based robust controllers for rigid robots",
% In Proc. First IEEE International Conference on Robotic Computing,
% 2017, to be published. 
%
% Author:       Andrea Giusti
% Written:      14-12-2015
% Last update:  05-03-2017
% Last revision:---

%------------- BEGIN CODE --------------

function [tao_out] = NE_mod(q,dq,dqA,dq_d,ddq,KinPar,DynPar,g)
%#codegen
% This function performs the modified recursive NE algorithm

DH_ext = KinPar.DHext;
B      = KinPar.B(1:3,1:3);
NJ = KinPar.NJ;
JT = DH_ext(:,7);

n= NJ;

% init recursion variables
r_k_km1k   = zeros(3,length(DynPar));
r_k_kck    = zeros(3,length(DynPar));
R_km1_k    = zeros(length(DynPar),3,3);
tR_km1_k   = zeros(length(DynPar),3,3);
omega_k_k  = zeros(3,length(DynPar));
omegaA_k_k = zeros(3,length(DynPar));
domega_k_k = zeros(3,length(DynPar));
ddp_k_k    = zeros(3,length(DynPar));
ddp_k_ck   = zeros(3,length(DynPar));
tao        = zeros(length(DynPar),1);
f_ip1_ip1  = zeros(3,1);
mu_ip1_ip1 = zeros(3,1);
z_0        = [0;0;1]; 
theta_i = 0;
%d_i = 0;

% Calculate rotation matrices for frame i to i-i and i+1 to i
count = 0;
while count < n
    count = count+1;
   
    a_i     = DH_ext(count,1); 
    alpha_i = DH_ext(count,2); 
    gamma_i = DH_ext(count,3);
    d_i     = DH_ext(count,4); 
    
    if DH_ext(count,7) == 0 % jt is rev
        theta_i = gamma_i + q(count);
    elseif  DH_ext(count,7) == 1 % jt is prism
        d_i = d_i + q(count);
        theta_i = gamma_i;
    end
    
    %r_dd = extDH2Rdd(DH_ext(count,:),q(count)); % as in the papers
    r_dd = [a_i;d_i*sin(alpha_i);d_i*cos(alpha_i)];% as in the dissertation

    r_k_km1k(:,count) = r_dd;

    r_k_kck(:,count) = DynPar(count).r_com;
    
    A_im1_ipr = [cos(theta_i), -sin(theta_i), 0, 0;
        sin(theta_i), cos(theta_i), 0, 0;
        0, 0, 1, d_i;
        0, 0, 0, 1];
    A_ipr_i   = [1, 0, 0, a_i;
        0, cos(alpha_i), -sin(alpha_i), 0;
        0, sin(alpha_i), cos(alpha_i) , 0;
        0, 0, 0, 1];
    if count == 1
        A_im1_i = A_im1_ipr*A_ipr_i;
    else
        A_im1_i = A_im1_ipr*A_ipr_i;
    end
    R_km1_k(count,:,:) = A_im1_i(1:3,1:3);
    tR_km1_k(count,:,:) = transpose(A_im1_i(1:3,1:3));
end
R_km1_k(count+1,:,:) = eye(3);

%Forward Recursion
%init
omega_i_i  = zeros(3,1);
omegaA_i_i = zeros(3,1);
domega_i_i = zeros(3,1);
omega_0_0 = zeros(3,1);
domega_0_0 = zeros(3,1);
ddp_0_0 = zeros(3,1)-transpose(B)*[0;0;g];
ddp_im1_im1 = zeros(3,1);
ddp_i_i =zeros(3,1);
omega_im1_im1 = zeros(3,1);
omegaA_im1_im1 = zeros(3,1);
domega_im1_im1 = zeros(3,1);
dtheta_i  = 0;
dthetaA_i = 0;
ddtheta_i = 0;
ddd_i =0;
dd_i =0;
ddA_i = 0;

% Forward recursion
count = 0;
while count < n
    count = count+1;
    %Initial conditions
    if count == 1
        z_0 = [0;0;1];
        %basis arm positioning
        omega_im1_im1 = omega_0_0;
        % Addition for modified NEalgo
        omegaA_im1_im1 = omega_0_0;
        domega_im1_im1 = domega_0_0;
        ddp_im1_im1 = ddp_0_0;      
    end
    
    R_i_ip1 = squeeze(R_km1_k(count+1,:,:));
    tR_im1_i = squeeze(tR_km1_k(count,:,:));
    
    if JT(count) == 0
        theta_i = q(count);
        dtheta_i = dq(count);
        % Addition for modified NE algo
        dthetaA_i = dqA(count);  
        ddtheta_i = ddq(count);
    end
    
    if JT(count)== 1
        d_i = q(count);
        dd_i = dq(count);
        % Addition for modified NE algo
        ddA_i = dqA(count);  
        ddd_i = ddq(count);
    end
    
    r_i_im1i = r_k_km1k(:,count);
    r_i_ici = r_k_kck(:,count);
    
    %Angular velocity calculation
    if JT(count) == 1
        %prismatic
        omega_i_i  = tR_im1_i*omega_im1_im1;
        omegaA_i_i = tR_im1_i*omegaA_im1_im1;
    elseif JT(count) == 0
        %rotoidal
        omega_i_i = tR_im1_i*(omega_im1_im1+dtheta_i*z_0);
        % addition for modified NEalgo
        omegaA_i_i = tR_im1_i*(omegaA_im1_im1+dthetaA_i*z_0);
    end
    
    %Angular acceleration calculation
    if JT(count) == 1
        %prismatic
        domega_i_i = tR_im1_i*domega_im1_im1;
    elseif JT(count) == 0
        %rotoidal
        % domega_i_i = tR_im1_i*(domega_im1_im1 + ddtheta_i*z_0 + cross(dtheta_i*omega_im1_im1,z_0));
        % modified for NE modiifed 1110
        domega_i_i = tR_im1_i*(domega_im1_im1 + ddtheta_i*z_0 + cross(dthetaA_i*omega_im1_im1,z_0));
    end
   
    %Acceleration of i
    if JT(count) == 1
        %prismatic
        ddp_i_i = tR_im1_i*(ddp_im1_im1 + ddd_i*z_0) + cross(dd_i*omegaA_i_i,tR_im1_i*z_0)+ cross(ddA_i*omega_i_i,tR_im1_i*z_0) + cross(domega_i_i,r_i_im1i) + cross(omegaA_i_i,cross(omega_i_i,r_i_im1i));
    elseif JT(count) == 0
        %rotoidal
        %ddp_i_i = tR_im1_i*ddp_im1_im1 + cross(domega_i_i,r_i_im1i) + cross(omega_i_i,cross(omega_i_i,r_i_im1i));
        %Modified for NE modified 1110
        ddp_i_i = tR_im1_i*ddp_im1_im1 + cross(domega_i_i,r_i_im1i) + cross(omegaA_i_i,cross(omega_i_i,r_i_im1i));
    end
    
    %Acceleration of the CoM i
    %Modified by the NE algo 1110
    ddp_i_ci = ddp_i_i + cross(domega_i_i,r_i_ici) + cross(omegaA_i_i,cross(omega_i_i,r_i_ici)); 
    
    omega_k_k(:,count) = omega_i_i;
    %addition for modified NE algo
    omegaA_k_k(:,count) = omegaA_i_i;
    domega_k_k(:,count) = domega_i_i;
    ddp_k_k(:,count) = ddp_i_i;
    ddp_k_ck(:,count) = ddp_i_ci;
    omega_im1_im1 = omega_i_i;
    %addition for modified NE algo
    omegaA_im1_im1= omegaA_i_i;
    domega_im1_im1 = domega_i_i;
    ddp_im1_im1 = ddp_i_i;
    
end

% Backward recursion
while count > 0
    r_i_im1i = r_k_km1k(:,count);
    r_i_ici = r_k_kck(:,count);
    R_i_ip1 = squeeze(R_km1_k(count+1,:,:));
    tR_im1_i = squeeze(tR_km1_k(count,:,:));
    omega_i_i = omega_k_k(:,count);
    %addition for modified NE algo
    omegaA_i_i = omegaA_k_k(:,count);
    domega_i_i = domega_k_k(:,count);
    
    m_i  = DynPar(count).m;
    %I_i  = DynPar(count).I-m_i*transpose(Skew(r_i_ici))*Skew(r_i_ici); %as in the paper
    I_i  = DynPar(count).I; %as in the disseration
    Fv_i = DynPar(count).DJ.jbv;
    Fc_i = DynPar(count).DJ.jbc;
    Kri  = DynPar(count).DJ.k_r;
    I_mi = DynPar(count).DJ.Im;
    
    if count == n
        %forces and torques at the end effector
        f_ip1_ip1 = zeros(3,1);
        mu_ip1_ip1 = zeros(3,1);
    end
    
   
    
    %Newton balance
    f_i_i = R_i_ip1*f_ip1_ip1 + m_i*ddp_k_ck(:,count);
    
    %Eulero balance
    %Modified by the NE algo 1110
    mu_i_i = -cross(f_i_i,r_i_im1i+r_i_ici) + R_i_ip1*mu_ip1_ip1 + cross(R_i_ip1*f_ip1_ip1,r_i_ici) + I_i*domega_i_i + cross(omega_i_i,I_i*omegaA_i_i);
    
    
    %Calculation of the generalized force at the joints
    if JT(count) == 1
        %prismatic
        tao(count) = transpose(f_i_i)*tR_im1_i*z_0 + Fv_i*dq_d(count)+...
                     Fc_i*sin(atan(1000*dq_d(count)))+ddq(count)*Kri^2*I_mi;
    elseif JT(count) == 0
        %rotoidal
        tao(count) = transpose(mu_i_i)*tR_im1_i*z_0 + Fv_i*dq_d(count)+...
                     Fc_i*sin(atan(1000*dq_d(count)))+ddq(count)*Kri^2*I_mi;
    end
    
    mu_ip1_ip1 = mu_i_i;
    f_ip1_ip1 = f_i_i;
   
    count = count-1;
end

tao_out = tao(1:NJ);

end