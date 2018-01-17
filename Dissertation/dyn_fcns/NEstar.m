% NEstar -  Performs the modified NE algorithm
%           !!! only Rev. Joints are allowed by now !!!
% Syntax:  
%    NEstar(eDH, Dyn_Data, gravity_vec, arrayQ, arrayQa, isnumeric,B)
%
% Inputs: 
%    eDH       -> eDH = cell array Nx5 alpha, a, d, theta, sigma, n. if prismatic, sigma=1, else sigma=0;
%  Dyn_Data    -> matr ix Nx11. mass (link+motor), [Ixx Ixy Ixz Iyy Iyz Izz] (link), CoM position, D    
% gravity_vec  -> gravitational acceleration vector
%   arrayQ     -> [q dq ddq]
%   arrayQa    -> [q_a dq_a ddq_a]
%  isnumeric   -> 1 for numerical computations, 0 for symbolic ones.
%     B        -> rotation matrix from the robot base frame to world ref. frame.
% Outputs:
% tau_e ->  vector of the generalized forces/torques at the joints 
%
% Example: 
%
% Other m-files required: ..tbf..
%                         ..tbf..                       
% Subfunctions: none
% MAT-files required: none
%
% See also: 
% A. De Luca, L. Ferrajoli, "A modified newton-euler method for 
% dynamic computations in robot fault detection and control", ICRA 2009. Algorithm version "0001". 
%
% Author:       Andrea Giusti
% Written:      27-September-2016
% Last update:  ---
% Last revision:---

%------------- BEGIN CODE --------------

function tau_e = NEstar(eDH, Dyn_Data, gravity_vec, arrayQ, arrayQa, isnumeric,B)
%#codgen

% ---Init--- 
   nlink = size(eDH, 1);
   %utility for interval checks
   idty = 0*Dyn_Data(1,1)+1; % added just for debug purpose, can be removed
    switch isnumeric
        case 0
           omega_init   = sym(zeros(3,1));
           omega_a_init = sym(zeros(3,1)); % auxiliary
           gamma_init   = sym(zeros(3,1));
		   tau_e = sym(zeros(nlink,1));
       case 1
           omega_init   = idty*zeros(3,1);
           omega_a_init = idty*zeros(3,1); % auxiliary
           gamma_init   = idty*zeros(3,1);
		   tau_e = idty*zeros(nlink,1);
   end
   acc_init  = -transpose(B)*gravity_vec*idty;
   z0 = [0;0;1];
  
% ---Reassigning dynamic parameters---
    % mass
    m_t = Dyn_Data(:,1);
    %Diagonal of inertia matrix of the links
    %xx xy xz yy yz zz
    I_all = Dyn_Data(:,2:7);
    % init Inertia matrices
    switch isnumeric
        case 0
            I_t=sym(zeros(3,3,nlink));
        case 1
            I_t=idty*zeros(3,3,nlink);
    end
    % fill inertia matrix
    for k=1:nlink
        I_t(:,:,k)=[...
            I_all(k,1), I_all(k,2), I_all(k,3);
            I_all(k,2), I_all(k,4), I_all(k,5);
            I_all(k,3), I_all(k,5), I_all(k,6)];
    end
    % COMs
    pos_com = Dyn_Data(:,8:10);
    %viscous damping
    Dq_t = Dyn_Data(:,11);
    
% ---Auxiliary structs definition---
    % ALL these quantities have an offset of 1; e.g: omega_1 = omega(:,1+1) = omega(:,2)
    % Angular velocity, acceleration
    omega   = idty*[omega_init, zeros(3, nlink)];
    omega_a = idty*[omega_a_init, zeros(3, nlink)]; % auxiliary
    gamma   = idty*[gamma_init, zeros(3, nlink)];
    switch isnumeric
        case 0
            % Linear acceleration of the frames
            acc  = sym([acc_init,  zeros(3, nlink)]);
            % Linear acceleration of the centers of mass
            acc_com  = sym(zeros(3, nlink+1));
        case 1
            % Linear acceleration, jerk and snap of the frames
            acc  = idty*[acc_init, zeros(3, nlink)];
            % Linear acceleration, jerk and snap of the centers of mass
            acc_com  = idty*zeros(3, nlink+1);
    end
    
    
% ---Retrieving q and its derivatives---
    q = arrayQ(:,1);
    qd = arrayQ(:,2);
    qdd = arrayQ(:,3);
    q_a_d = arrayQa(:,2);
    
% ---Computing DH matrix and joint variables---
    
    switch isnumeric
        case 0
            % t stands for total
            d_t = sym(zeros(nlink, 1));
            d_d_t = sym(zeros(nlink, 1));
            d_dd_t = sym(zeros(nlink, 1));
            theta_t = sym(zeros(nlink, 1));
            theta_d_t = sym(zeros(nlink, 1));
            theta_dd_t = sym(zeros(nlink, 1));
            
            R_all=sym(zeros(3,3,nlink+1));
            p_all=sym(zeros(3,1,nlink+1)); % this holds the ^i p_{i,i-1}
        case 1
            % t stands for total
            d_t = zeros(nlink, 1);
            d_d_t = zeros(nlink, 1);
            d_dd_t = zeros(nlink, 1);
            theta_t = zeros(nlink, 1);
            theta_d_t = zeros(nlink, 1);
            theta_dd_t = zeros(nlink, 1);
            theta_a_d_t = zeros(nlink, 1);
            
            R_all=zeros(3,3,nlink+1);
            p_all=zeros(3,1,nlink+1); % this holds the ^i p_{i,i-1}
    end
    for i=1:nlink

        %Setting up d's, theta's, and the transformation matrix
        alpha = eDH(i,1);      % alpha_i
        a = eDH(i,2);          % a_i                        
        if(eDH(i,5)==0) % joint i is revolute
            d_t(i) = eDH(i,3);       % d_i
            theta_t(i) = q(i) + eDH(i,4);      % theta_i
            theta_d_t(i) = qd(i);
            theta_dd_t(i) = qdd(i);
            theta_a_d_t(i) = q_a_d(i); 
        else  % is prismatic
            d_t(i) = q(i);          % d_i
            d_d_t(i) = qd(i);
            d_dd_t(i) = qdd(i);
            theta_t(i) = eDH(i,4);   % theta_i
        end
        ct = cos(theta_t(i));
        st = sin(theta_t(i));
        ca = cos(alpha);
        sa = sin(alpha);
        % Rotation matrix for standard DH notation
        R = [  ct,  -ca*st,   sa*st;
               st,   ca*ct,  -sa*ct;
                0,      sa,      ca];
        R_all(:,:,i)=R;
        p_all(:,i) = [a;d_t(i)*sa;d_t(i)*ca+eDH(i,6)]; % DH(i,6) := n_i
    end
    R_all(:,:,nlink+1)=eye(3); % this ok? what if fn_ext, fn_d_ext, fn_dd_ext != 0 ? should be {}^nlink T_0 ?
    % at the end of this loop:
    % {}^{i-1} R_i = R_all(:,:,i)
    % {}^i p_{i,i-1} = p_all(:,i)
    % e.g: R_all(:,:,1) = {}^0 R_1 and p_all(:,1) = {}^1 p_{1,0}
	
    % --- Forward recursion
    for k=1:nlink
        R=R_all(:,:,k); % this is {}^{k-1} R_k
        Rt=R.';
        p=p_all(:,k);
        pc=pos_com(k,:).'; % this is {}^k p_{c_k,k}
        i=k+1;
        i1=k;
        %zi=Rt(:,3); % this is {}^k \hat{z}_{k-1}

        if(eDH(k,5)==0) % i.e: joint k is revolute
            theta_d=theta_d_t(k);
            theta_a_d = theta_a_d_t(k); % auxiliary
            theta_dd=theta_dd_t(k);

            omega(:,i) = Rt*( omega(:,i1) + theta_d*z0 );
            omega_a(:,i) = Rt*( omega_a(:,i1) + theta_a_d*z0 ); % auxiliary
            gamma(:,i) = Rt*( gamma(:,i1) + theta_dd*z0 + cross(omega_a(:,i1),theta_d*z0) ); % AAR0
            omegap_a = cross(omega_a(:,i),p); % LAR'0
            oomegap = cross(omega(:,i),omegap_a);  

            acc(:,i)  = Rt*acc(:,i1)  + cross(gamma(:,i),p) + oomegap;   %%%               
%         else % is prismatic
%             d_dzi=d_d_t(k)*zi;
%             d_ddzi=d_dd_t(k)*zi;             
% 
%             omega(:,i) = Rt*omega(:,i1);
%             gamma(:,i) = Rt*gamma(:,i1);
% 
%             omegap = cross(omega(:,i),p);
%             oomegap = cross(omega(:,i),omegap);
%             omegad_dzi = cross(omega(:,i),d_dzi);
% 
%             acc(:,i)  = Rt*acc(:,i1) + cross(gamma(:,i),p) + oomegap ...
%                 + 2*omegad_dzi + d_ddzi;
        end

        omegapc_a = cross(omega_a(:,i),pc); % LAR0
        oomegapc = cross(omega(:,i),omegapc_a);          
        
        acc_com(:,i)  = acc(:,i) + cross(gamma(:,i),pc) + oomegapc;     
    end % --- end of forward recursion

    % --- Backward recursion
    f = zeros(3,1);
    n = zeros(3,1);
    % remember: {}^k R_{k+1} = R_all(:,:,k+1)
    % {}^k p_{k,k-1} = p_all(:,k)
    for k=nlink:-1:1
        R=R_all(:,:,k+1);  % this is {}^k R_{k+1}
        p=p_all(:,k);      % this is {}^k p_{k,k-1}
        zi=R_all(3,:,k).'; % this is {}^k \hat{z}_{k-1} = {}^k R_{k-1}*z0 = ( {}^{k-1}R_k )^T *z0
        m=m_t(k);
        I=I_t(:,:,k);  %%% build the interval from infimum and supremum to test correctness
        Dq=Dq_t(k);
        pc=pos_com(k,:).';
        i=k+1;
        F    = m*acc_com(:,i);
        N = I*gamma(:,i) + cross(omega_a(:,i),I*omega(:,i)); % TR1
        f    = R*f    + F; % Rba*fb, Rba si calcola come Ra*inv(Rb)
        n    = R*n + cross(pc,F) + cross(p,f) + N; % Rba*nb + Rba*cross(pb,fb)
        if(eDH(k,5)==0)
            tau_e(k) = n.'*zi + Dq*qd(k);
        else
            tau_e(k) = f.'*zi + Dq*qd(k);
        end      
    end % --- end of backward recursion