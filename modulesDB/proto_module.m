% proto_module - Returns the prototype structure of a simple module
%
% Syntax: proto_module()  
%
% Inputs: 
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
% Written:      14-12-2015
% Last update:  05-03-2017
% Last revision:---

%------------- BEGIN CODE --------------

function [Module] = proto_module()
% proto_Module simply returns the prototype of a module with zeros in all
% its entries

% ID: unique module identifier, typ: type(ee 3, link 2, joint 1),
% nb: we use the same structure for joint and link modules for simplicity
%     in this case we store the corresponding info in the proximal part
%     data location (except delta_l,out which is in the distal part 
%     data location)
%     
% Cplx: complexity (number of elementary units [1] required)
% CANidTX/RX: CAN bus address for TX and RX of CAN messages
Module.Mod = struct('ID'  ,0,'typ',0,'Cplx',0,'CANidTX',0,'CANidRX',0);
Module.Kpl = struct('a_pl',0,'alpha_pl',0,'p_pl',0,'n_pl',0,'delta_pl',0);
Module.Kdl = struct('a_dl',0,'alpha_dl',0,'p_dl',0,'n_dl',0,'delta_dl',0);
% jt: joint type, 0-rev. 1-prism., Ujl/Ljl: upper/lower joint limit
Module.Kj  = struct('jt'  ,0,'delta_j' ,0,'Ujl' ,0,'Ljl' ,0);
Module.Dpl = struct('m_pl',0,'I_pl' ,zeros(3,3),'rcom_pl',zeros(3,1));
Module.Ddl = struct('m_dl',0,'I_dl' ,zeros(3,3),'rcom_dl',zeros(3,1));
% jbv: viscous friction coeff, jbc: Coulomb friction coeff,
% k_tau: motor torque constant, k_r: gear reduction ratio, 
% tau_lim: joint torque/force limit, curr_lim: mA curr limit
% dq_lim: joint vel. limit, ddq_lim: joint acc. limit,
% Ke: Elastic constant for elastic-joint modules (0 when joint is stiff)
Module.Dj  = struct('Im'  ,0,'jbv',0,'jbc',0,'Ke',0,'k_tau',0,'k_r',0,'tau_lim',0,'curr_lim',0,'dq_lim',0,'ddq_lim',0);
% see [1] for further detail on the parameters

end

