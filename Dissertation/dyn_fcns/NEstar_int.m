% NEstar_int -  Performs the Interval-arithmetic-based NE algorithm (numerically)
%            
% Syntax:  
%    NEstar_int(eDH, Dyn_Data_Inf, Dyn_Data_Sup, gravity_vec, arrayQ, arrayQa,B)
%
% Inputs: 
%    eDH       -> eDH = cell array Nx5 alpha, a, d, theta, sigma, n. if prismatic, sigma=1, else sigma=0;
% Dyn_Data_Inf -> Inf. of matrix Nx14. mass (link+motor), [Ixx Ixy Ixz Iyy Iyz Izz] (link), CoM position, Bv, Bc, Im  sigma_r  
% Dyn_Data_Sup -> Sup. of matrix Nx14. mass (link+motor), [Ixx Ixy Ixz Iyy Iyz Izz] (link), CoM position, Bv, Bc, Im, sigma_r
% gravity_vec  -> gravitational acceleration vector
%   arrayQ     -> [q dq ddq]
%   arrayQa    -> [0 dq_a 0]
%     B        -> rotation matrix from the robot base frame to world ref. frame.
% Outputs:
% [tau_Inf, tau_Sup] -> Inf. and Sup. of the interval vectors of the generalized forces/torques at the joints 
%
% Example: 
%
% Other m-files required: ..tbf..
%                         ..tbf..                       
% Subfunctions: none
% MAT-files required: sum_Int(), sub_Int(), prod_Int(), prodVV_Int(), prodSV_Int(), prodMV_Int() 
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

function [tau_Inf, tau_Sup] = NEstar_int(eDH, Dyn_Data_Inf, Dyn_Data_Sup, gravity_vec, arrayQ, arrayQa,B)
%#codgen
    
% ---Init--- 
	omega_init   = zeros(3,1);
    omega_a_init = zeros(3,1); % auxiliary
    gamma_init   = zeros(3,1);
	acc_init  = -transpose(B)*gravity_vec;
    z0 = [0;0;1];
    nlink = size(eDH, 1);
	% init output
    tau_Inf = zeros(nlink,1);
    tau_Sup = zeros(nlink,1);
% ---Reassigning dynamic parameters---
    % mass Inf. and Sup.
    m_t_Inf = Dyn_Data_Inf(:,1);
    m_t_Sup = Dyn_Data_Sup(:,1);
    % diagonal of inertia matrix of the links Inf. and Sup.
    % xx xy xz yy yz zz
    I_all_Inf = Dyn_Data_Inf(:,2:7);
    I_all_Sup = Dyn_Data_Sup(:,2:7);
	% init Inertia matrices Inf. and Sup.
    I_t_Inf = zeros(3,3,nlink);
    I_t_Sup = zeros(3,3,nlink);  
	% fill inertia matrices Inf. and Sup.
    for k=1:nlink
        I_t_Inf(:,:,k)=[...
            I_all_Inf(k,1), I_all_Inf(k,2), I_all_Inf(k,3);
            I_all_Inf(k,2), I_all_Inf(k,4), I_all_Inf(k,5);
            I_all_Inf(k,3), I_all_Inf(k,5), I_all_Inf(k,6)];
        I_t_Sup(:,:,k)=[...
            I_all_Sup(k,1), I_all_Sup(k,2), I_all_Sup(k,3);
            I_all_Sup(k,2), I_all_Sup(k,4), I_all_Sup(k,5);
            I_all_Sup(k,3), I_all_Sup(k,5), I_all_Sup(k,6)];
    end 
    % COMs Inf. and Sup. 
    pos_com_Inf = Dyn_Data_Inf(:,8:10);   
    pos_com_Sup = Dyn_Data_Sup(:,8:10);   
    % viscous damping  Inf. and Sup.
    Bv_t_Inf = Dyn_Data_Inf(:,11);
    Bv_t_Sup = Dyn_Data_Sup(:,11);
    % static friction  Inf. and Sup.
    Bc_t_Inf = Dyn_Data_Inf(:,12);
    Bc_t_Sup = Dyn_Data_Sup(:,12);
    % rotor inertia Inf. and Sup.
    Im_t_Inf = Dyn_Data_Inf(:,13);
    Im_t_Sup = Dyn_Data_Sup(:,13);
    % gear ration non-uncertain
    sigma_t_r = Dyn_Data_Inf(:,14);%+Dyn_Data_(:,14);
    
% ---Auxiliary structs definition---
    % ALL these quantities have an offset of 1; e.g: omega_1 = omega(:,1+1) = omega(:,2)
    % Angular velocity, acceleration
    omega   = [omega_init, zeros(3, nlink)];
    omega_a = [omega_a_init, zeros(3, nlink)]; % auxiliary
    gamma   = [gamma_init, zeros(3, nlink)];
    % linear accelerationof the frames
    acc  = [acc_init,  zeros(3, nlink)];
    % linear acceleration the centers of mass Inf. and Sup.
    acc_com_Inf  = zeros(3, nlink+1);
    acc_com_Sup  = zeros(3, nlink+1);
    
% ---Retrieving q and its derivatives---
    q  = arrayQ(:,1);
    dq  = arrayQ(:,2);
    ddq = arrayQ(:,3);
    dq_a  = arrayQa(:,2);
	
% ---Computing DH matrix and joint variables---
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
    for i=1:nlink
        %Setting up d's, theta's, and the transformation matrix
        alpha = eDH(i,1);      % alpha_i
        a     = eDH(i,2);      % a_i
        if(eDH(i,5)==0) % joint i is revolute
            d_t(i) = eDH(i,3);       % d_i
            theta_t(i)    = q(i) + eDH(i,4);      % theta_i
            theta_d_t(i)  = dq(i);
            theta_dd_t(i) = ddq(i);
            theta_a_d_t(i)  = dq_a(i);
         else  % is prismatic
              d_t(i) = q(i)+ eDH(i,3);          % d_i
              d_d_t(i) = dq(i);
              d_dd_t(i) = ddq(i);
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
        R_all(:,:,i) = R;
        p_all(:,i)   = [a;d_t(i)*sa;d_t(i)*ca+eDH(i,6)]; % DH(i,6) := n_i
    end
    R_all(:,:,nlink+1) = eye(3); % this ok? what if fn_ext, fn_d_ext, fn_dd_ext != 0 ? should be {}^nlink T_0 ?
    % at the end of this loop:
    % {}^{i-1} R_i = R_all(:,:,i)
    % {}^i p_{i,i-1} = p_all(:,i)
    % e.g: R_all(:,:,1) = {}^0 R_1 and p_all(:,1) = {}^1 p_{1,0}

    % ---Forward recursion---
    for k=1:nlink
        R=R_all(:,:,k); % this is {}^{k-1} R_k
        Rt=R.';
        p=p_all(:,k);
        pc_Inf = pos_com_Inf(k,:).'; % this is {}^k p_{c_k,k}
        pc_Sup = pos_com_Sup(k,:).'; % this is {}^k p_{c_k,k}
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
            
            acc(:,i)  = Rt*acc(:,i1)  + cross(gamma(:,i),p) + oomegap;
        end

        [omegapc_a_Inf, omegapc_a_Sup] = cross_Int(omega_a(:,i),omega_a(:,i),pc_Inf,pc_Sup); % LAR0
        [oomegapc_Inf, oomegapc_Sup] = cross_Int(omega(:,i),omega(:,i),omegapc_a_Inf,omegapc_a_Sup);          
        
        [cr_Inf , cr_Sup] = cross_Int(gamma(:,i),gamma(:,i),pc_Inf,pc_Sup);
        [s1_Inf, s1_Sup] = sum_Int(acc(:,i),acc(:,i), cr_Inf, cr_Sup);
        [acc_com_Inf(:,i), acc_com_Sup(:,i)] = sum_Int(s1_Inf,s1_Sup, oomegapc_Inf, oomegapc_Sup);
              
    end % --- end of forward recursion

    % ---Backward recursion---
    % init the external forces Inf. and Sup.
	f_Inf = zeros(3,1);
    n_Inf = zeros(3,1);
    f_Sup = zeros(3,1);
    n_Sup = zeros(3,1);
    
    % remember: {}^k R_{k+1} = R_all(:,:,k+1)
    % {}^k p_{k,k-1} = p_all(:,k)
    for k = nlink:-1:1
        R  = R_all(:,:,k+1);  % this is {}^k R_{k+1}
        p  = p_all(:,k);      % this is {}^k p_{k,k-1}
        zi = R_all(3,:,k).'; % this is {}^k \hat{z}_{k-1} = {}^k R_{k-1}*z0 = ( {}^{k-1}R_k )^T *z0
        m_Inf = m_t_Inf(k);
        m_Sup = m_t_Sup(k);
        I_Inf  = I_t_Inf(:,:,k);
        I_Sup  = I_t_Sup(:,:,k);
        Bv_Inf = Bv_t_Inf(k);
        Bv_Sup = Bv_t_Sup(k);
        Bc_Inf = Bc_t_Inf(k);
        Bc_Sup = Bc_t_Sup(k);
        Im_Inf = Im_t_Inf(k);
        Im_Sup = Im_t_Sup(k);
        sigma_r = sigma_t_r(k);
        pc_Inf = pos_com_Inf(k,:).'; % this is {}^k p_{c_k,k}
        pc_Sup = pos_com_Sup(k,:).'; % this is {}^k p_{c_k,k}
        i  = k+1;
        
		%F = m*acc_com(:,i); 
        [F_Inf, F_Sup] = prodSV_Int(m_Inf,m_Sup,acc_com_Inf(:,i),acc_com_Sup(:,i));
        
        %N = I*gamma(:,i) + cross(omega_a(:,i),I*omega(:,i)); % TR1
        %       -p1-                              -p2-
        %                                   -c1-
        [p1_Inf, p1_Sup] = prodMV_Int(I_Inf,I_Sup,gamma(:,i),gamma(:,i));
        [p2_Inf, p2_Sup] = prodMV_Int(I_Inf,I_Sup,omega(:,i),omega(:,i));
        [c1_Inf, c1_Sup] = cross_Int(omega_a(:,i),omega_a(:,i),p2_Inf, p2_Sup);
        [N_Inf, N_Sup]   = sum_Int(p1_Inf, p1_Sup,c1_Inf, c1_Sup);
        
        %f    = R*f    + F;
        %      -p3-
        [p3_Inf,p3_Sup] = prodMV_Int(R,R,f_Inf,f_Sup);
        [f_Inf,f_Sup]   = sum_Int(p3_Inf,p3_Sup,F_Inf,F_Sup);
        
        %n    = R*n + cross(pc,F) + cross(p,f) + N;
        %       -p4-     -c2-           -c3-    
        %           -s1-         
        %                        -s2-
        [p4_Inf,p4_Sup]  = prodMV_Int(R,R,n_Inf,n_Sup);
        [c2_Inf,c2_Sup]  = cross_Int(pc_Inf,pc_Sup,F_Inf,F_Sup);
        [c3_Inf,c3_Sup]  = cross_Int(p,p,f_Inf,f_Sup);
        [s1_Inf,s1_Sup]  = sum_Int(p4_Inf,p4_Sup,c2_Inf,c2_Sup);
        [s2_Inf,s2_Sup]  = sum_Int(s1_Inf,s1_Sup,c3_Inf,c3_Sup);
        [n_Inf,n_Sup]    = sum_Int(s2_Inf,s2_Sup,N_Inf,N_Sup);
           
        if(eDH(k,5)==0)
            % tau_e(k) = n.'*zi + Bv*qd(k) +  Bc*sign(qd(k)) + Im*sigma_r^2*ddq;
            %             -p5-     -p6-        -p7-               -p8-
            %                 -s3-
            %                             -s4-
            [p5_Inf, p5_Sup] = prodVV_Int(n_Inf',n_Sup',zi,zi);
            [p6_Inf, p6_Sup] = prod_Int(Bv_Inf,Bv_Sup,dq(k),dq(k));
            [p7_Inf, p7_Sup] = prod_Int(Bc_Inf,Bc_Sup,sign(dq(k)),sign(dq(k)));
            [p8_Inf, p8_Sup] = prod_Int(Im_Inf,Im_Sup,ddq(k)*sigma_r^2,ddq(k)*sigma_r^2);
            [s3_Inf, s3_Sup] = sum_Int(p5_Inf,p5_Sup,p6_Inf,p6_Sup);
            [s4_Inf, s4_Sup] = sum_Int(s3_Inf,s3_Sup,p7_Inf,p7_Sup);
            
            [tau_Inf(k), tau_Sup(k)] = sum_Int(s4_Inf,s4_Sup,p8_Inf,p8_Sup);
%         else
%             tau_e(k) = f.'*zi + Dq*qd(k);
        end        
    end % --- end of backward recursion