% invkin_d - Dynamic inverse kinematics using unit quarternions
%
% Syntax:  
%    dq = invkin_2d(DHtab,jt,t_samp,q_curr,p_d,dp_d,R_d,omega_d,jlimit,mode);
%
% Inputs: 
%    DHtab                     -> Standard DH table, the i-th row of DHtab is
%                                       [a_i, alpha_i, gamma_i, d_i].
%    jt                        -> Id vector of joint types (0 for rev., 1 for prism.)
%    t_samp (double)           -> sampling time
%    q_curr (n x 1 matrix)     -> current joint position
%    p_d    (3 x 1 matrix)     -> desired cartesian position
%    dp_d   (3 x 1 matrix)     -> desired cartesian velocity
%    R_d    (3 x 3 rot.-matrix)-> desired cartesian orientation 
%    omega_d(3 x 3 rot.-matrix)-> desired cartesian angle velocities 
%    jlimit (n x 2 matrix)     -> matrix of joint limits, the i-th row of jl is [lim_lo, lim_up]
%    mode   (string)           -> case 'inverse' (Default)
%                                 case 'transpose': no joint limit and
%                                 no singularity avoidance
%
%
% Outputs:
%    dq   (n x 1 matrix)       -> resulting joint velocity 
%
% Example: 
%  
% Other m-files required:   Jacobian(q,DHtab,jt), fwdkin(q,DHtab,jt),rot2quat(R),Skew(w)
% Subfunctions:             DLS(J,K_inv)
% MAT-files required:       none
%
% See also: 
% [1] S. Chiaverini and B. Siciliano, "The Unit Quarternion: A Useful Tool for 
% Inverse Kinematics of Robot Manipulators" in "Systemw Analysis, Modelling and Simulation", 1999.
% [2] S. Chiaverini and B. Siciliano, "Review of the Damped Least-Squares Inverse 
% Kinematics with Experiments on an Industrial Robot Manipulator", 1994.

% Author:       Andrea Giusti
% Written:      ---
% Last update:  ---
% Last revision:---

%------------- BEGIN CODE --------------
function [ddq,e_pos,e_ori] = invkin_2d(DHtab,jt,t_samp,q_curr,dq_curr,p_d,dp_d,ddp_d,R_d,omega_d,domega_d,jlimit)
    %#codegen
    [N, ~]= size(DHtab);
    [dJ,J] = dJacobian(q_curr,dq_curr,DHtab,jt);

    %% CLIK Parameters
    alpha_inv = 0.7;
    K_inv = alpha_inv/t_samp;               % Gain for Inverse Jacobian Approach
    alpha_tra = 0.7;
    K_tra = alpha_tra/t_samp;               % Gain for Transposed Jacobian Approach
    k_null_rev = 100;                       % nullspace motion gain for rev. joints
    k_null_pris = 7;                        % nullspace motion gain for prism. joints
    
    %% Position and angle error
    [p,R] = fwdkin(q_curr,DHtab,jt);    %forward kinematics
    [eta_d,epsilon_d] = rot2quat(R_d);  %Desired Quaternions
    [eta,epsilon] = rot2quat(R);        %Actual Quaternions
    S_epsilon_d = Skew(epsilon_d);      
   
    
    de_omega = omega_d-J(4:6,:)*dq_curr;
    e_pos  = p_d-p;                    %Position error
    de_pos = dp_d-J(1:3,:)*dq_curr;    %d Position error
    e_ori = eta * epsilon_d - eta_d * epsilon - S_epsilon_d * epsilon; %Orient. error
    
    
    %% Maximize distance to joint limits in Nullspace
    dq_null = zeros(N,1);
    for i=1:N                           %partial derivative of distance
        jl_m = (jlimit(i,2) + jlimit(i,1))/2;   %middle value of i-th joint range
        k_null = k_null_rev*not(jt(i)) + k_null_pris*jt(i);
        dq_null(i) = k_null * (- 1/N) * (q_curr(i)-jl_m) / (jlimit(i,2) - jlimit(i,1))^2;
    end
    
    [J_inv,~] = DLS(J,K_inv);
    
    %% Inverse Kinematics 2 order
        % position error dynamics is a second order linear system 
        %Kp = omega_n^2; Kv = 2*zeta*omega_n
        omega_n = 100;
        Kp = omega_n^2;
        Kv = 2*omega_n;
        Komega = 10;
        Ko = 200*eye(3);
        z1_P = ddp_d + Kv*(de_pos) + Kp*e_pos;
        z1_O = domega_d + Komega*(de_omega) + Ko*e_ori;
        c = 200;
        ddq = J_inv*[z1_P;z1_O] - J_inv*dJ*dq_curr- c*(eye(N) - J_inv*J)*dq_curr; % the last term on the RHS introduces damping in the null space
end

%% Subfunction: Damped Least Squares Inverse Kinematics (in case of singularities)
function [J_inv_d,K_inv_d] = DLS(J,K_inv)
    sn = 0.04; %singular region
    lambda_max = 0.04;% it was 0.04; %should equal "sn"
    [~,S,~] = svd(J);
    s_min = min(diag(S)); %smallest singular value
    [~,N] = size(J);
    
    % 1. Basic DLS Scheme
    if (s_min >= sn) %outside singular region
        lambda_squared = 0;
    else %inside singular region
        lambda_squared = (1-(s_min/sn)^2) * lambda_max^2;
    end
    J_inv_d = pinv(J'*J + lambda_squared*eye(N))*J'; %Damped Inverse
    
    % 2. Feedback Correction
    if (s_min <= sn)
        rho = 1;%it was 0; %deactivate closed loop feedback in singular region
    elseif (s_min < 4*sn) && (s_min > sn)
        rho = ((s_min-sn)/(3*sn))^2; %4-order transition
    else
        rho = 1; %normal CLIK
    end
    K_inv_d = K_inv;% or rho*K_inv; %adjust CLIK gain
end