% invkin_s - Static inverse kinematics using unit quaternions
%
% Syntax:  
%    [q,info,info_text] = invkin_s(DHtab,jt,p_d)
%    [q,info,info_text] = invkin_s(DHtab,jt,p_d,R_d)
%    [q,info,info_text] = invkin_s(DHtab,jt,p_d,R_d,'init',q_0,'jlimit',jl,'mode','transposed','iter',50,'pos_th',0.001,'ori_th',0.001)
%
% Inputs:                      
%    DHtab                     -> Standard DH table, the i-th row of DHtab is [a_i, alpha_i, gamma_i, d_i].
%    jt                        -> Id vector of joint types (0 for rev., 1 for prism.)
%    p_d  (3 x 1 matrix)       -> Desired cartesian position in [m]
%    R_d  (3 x 3 rot.-matrix)  -> (optional) Desired cartesian orientation 
%
% Parameters:
%    init (n x 1 matrix)       -> initial joint position q_0 .                                      Default: zeros
%    mode (string)             -> 'inverse'/'transposed'.                                           Default: 'inverse'
%    jlimit (n x 2 matrix)     -> matrix of joint limits, the i-th row of jl is [lim_lo, lim_up].   Default: none
%    iter   (int)              -> number of iterations                                              Default: 100
%    pos_th (real)             -> Stopping Criteria: Error norm (Position) in [m]                   Default: 0.0001 
%    ori_th (real)             -> Stopping Criteria: Error norm (Orientation) in [rad].             Default: 0.0001 
%
% Outputs:
%    q    (n x 1 matrix)       -> resulting joint position
%    info                      -> 0: Success
%                                 1: no convergence (no solution or 
%                                    not global minimum)
%                                 2: convergence, but joint limits violated
%    info_text                 -> String message corresponding to "info"
%
% Units:
%    [m] for prismatic joints
%    [rad] for revolute joints [-pi;pi]
%
% Example:
%
% Other m-files required: Jacobian(q,DHtab,jt), forwkin(q,DHtab,jt), rot2quat(R),Skew(w)
% Subfunctions:       none
% MAT-files required: none
%
% See also: 
% [1] S. Chiaverini and B. Siciliano, "The Unit Quaternion: A Useful Tool for 
% Inverse Kinematics of Robot Manipulators" in "Systemw Analysis, Modelling and Simulation", 1999.

% Author:       Stefan Liu, Andrea Giusti
% Written:      18-January-2016
% Last update:  02-May-2017
% Last revision:---

%------------- BEGIN CODE --------------
function [q,info,info_text] = invkin_s(DHtab,jt,p_d,varargin)
    [N, ~]= size(DHtab);
    %% Input Parser
    inputparser = inputParser;
    R_d_default = 0;
    q_0_default = zeros(N,1);
    jl_default = 0;
    mode_default = 'inverse';
    iter_default = 100;
    th_pos_default = 0.0001;
    th_ori_default = 0.0001;

    addRequired(inputparser,'DHtab',@ismatrix);
    addRequired(inputparser,'jt',@ismatrix);
    addRequired(inputparser,'p_d',@(x) validateattributes(x,{'numeric'},{'size',[3,1]}));
    addOptional(inputparser,'R_d',R_d_default,@(x) validateattributes(x,{'numeric'},{'size',[3,3]}));
    addParameter(inputparser,'init',q_0_default,@(x) validateattributes(x,{'numeric'},{'size',[N,1]}));
    addParameter(inputparser,'jlimit',jl_default,@(x) validateattributes(x,{'numeric'},{'size',[N,2]}));
    addParameter(inputparser,'mode',mode_default,@(x) any(validatestring(x,{'inverse','transpose'})));
    addParameter(inputparser,'iter',iter_default,@isnumeric);
    addParameter(inputparser,'pos_th',th_pos_default,@isnumeric);
    addParameter(inputparser,'ori_th',th_ori_default,@isnumeric);
    parse(inputparser,DHtab,jt,p_d,varargin{:});
    
    R_d = inputparser.Results.R_d;
    q_0 = inputparser.Results.init;
    jl = inputparser.Results.jlimit;
    mode = inputparser.Results.mode;
    iter = inputparser.Results.iter;
    pos_th = inputparser.Results.pos_th;
    ori_th = inputparser.Results.ori_th;
    
    ori_info = not(isscalar(R_d));
    limit_info = not(isscalar(jl));
    
    if ori_info  
        [eta_d,epsilon_d] = rot2quat(R_d); % Desired Quaternions (if R_d given)
        S_epsilon_d = Skew(epsilon_d);      % Skew version of epsilon_d
    else
        e_ori = 0;
    end
    
    %% CLIK Parameters
    max_iterations = iter;                   % Max iterations of calculation (from inputParser)
    max_error_pos = pos_th;                  % loop-break condition position (from inputParser)
    max_error_ori = ori_th;                  % loop-break condition orientation (from inputParser)
    K_inv = 0.8;                            % Gain for Inverse Jacobian Approach
    K_tra = 0.5;                            % Gain for Transposed Jacobian Approach
    k_null_rev = 100;                       % nullspace motion gain for rev. joints
    k_null_pris = 7;                        % nullspace motion gain for prism. joints

    %% Loop
    dq_null = zeros(N,1);
    jlimit_violated = 0;
    violated_index = 0;
    q = q_0;
    c = 1;
    while 1
        %% Position and angle error
        [p,R] = fwdkin(q,DHtab,jt);   %Calculate Forward Position
        e_pos = p_d - p;                        %Calculate Position Error
        J = Jacobian(q,DHtab,jt);               %Calculate Jacobian
        if ori_info                             %Calculate Orientation Error
            [eta,epsilon] = rot2quat(R);       %Calculate Quaternions
            e_ori = eta * epsilon_d - eta_d * epsilon - S_epsilon_d * epsilon;     
            e = [e_pos;e_ori];
        else  %orientation not given
            J = J(1:3,:);
            e = e_pos;
        end
        
        %% Maximize distance to joint limits in Nullspace
        if limit_info                           %limits given
            for i=1:N                           %partial derivative of distance
                jl_m = (jl(i,2) + jl(i,1))/2;   %middle value of i-th joint range
                k_null = k_null_rev*not(jt(i)) + k_null_pris*jt(i);
                dq_null(i) = k_null * (- 1/N) * (q(i)-jl_m) / (jl(i,2) - jl(i,1))^2;
                if (q(i) > jl(i,2)) || (q(i) < jl(i,1))
                    jlimit_violated = 1;
                    violated_index = i;
                end
            end
            jlimit_violated = any(q > jl(:,2)) + any(q < jl(:,1));
        end    
        
        %% Inverse Kinematics
        if strcmp(mode,'inverse')
            J_inv = pinv(J);
            q = q + J_inv*K_inv*e + (eye(N) - J_inv*J)*dq_null;
        elseif strcmp(mode,'transpose')
            q = q + J'*K_tra*e;
        end
        
%         % wrap solution to [-pi;pi] for rev. joints
%         for i=1:N
%             if jt(i) == 0
%                 q(i) = wrapToPi(q(i));
%             end
%         end
        % wrap solution to [-pi;pi] for rev. joints usable for code gen
        for i=1:N
            if jt(i) == 0
                q(i) = mod(q(i),2*pi);
                if q(i) > pi
                    q(i) = q(i)-2*pi;
                end
            end
        end
        
        %% Break Conditions
        if (norm(e_pos) < max_error_pos) && (norm(e_ori) < max_error_ori) 
            if not(jlimit_violated)
                info = 0; %success
                info_text = 'success';
                break
            elseif c >= max_iterations
                info = 2; %converged, but joint limits violated
                info_text = strcat('joint limit violated at J ',int2str(violated_index));
                break
            end
        elseif c >= max_iterations
            info = 1; %no solution or not global minimum
            info_text = 'no solution';
            break
        end
        c = c+1;
        
    end
end