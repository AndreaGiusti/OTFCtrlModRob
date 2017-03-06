% init_SchunkModDB - Script for initializing the Modules DB
%
% Syntax:  
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

% Author:       Andrea Giusti
% Written:      14-12-2015
% Last update:  05-03-2017
% Last revision:---

%------------- BEGIN CODE --------------


% Initialize DB of the modules known by the control
%(units are meters, seconds, rads)
%init module struct prototype
% Module.Mod = struct('ID'  ,0,'typ',0,'Cplx',0,'CANidTX',0,'CANidRX',0);
% Module.Kpl = struct('a_pl',0,'alpha_pl',0,'p_pl',0,'n_pl',0,'delta_pl',0);
% Module.Kdl = struct('a_dl',0,'alpha_dl',0,'p_dl',0,'n_dl',0,'delta_dl',0);
% Module.Kj  = struct( 'jt' ,0,'delta_j' ,0,'Ujl' ,0,'Ljl' ,0);
% Module.Dpl = struct('m_pl',0,'I_pl' ,zeros(3,3),'rcom_pl',zeros(3,1));
% Module.Ddl = struct('m_dl',0,'I_dl' ,zeros(3,3),'rcom_dl',zeros(3,1));
% Module.Dj  = struct('Im'  ,0,'jbv',0,'jbc',0,'k_tau',0,'k_r',0,'tau_lim',0,'dq_lim',0,'ddq_lim',0);
Module = proto_module();

% Joint modules
%init Power Ball 1
PBl1.Mod = struct('ID'  ,1,'typ',1,'Cplx',2,'CANidTX',515,'CANidRX',387);
PBl1.Kpl = struct('a_pl',0,'alpha_pl',0,'p_pl',0,'n_pl',0,'delta_pl',0);
PBl1.Kdl = struct('a_dl',0,'alpha_dl',-pi/2,'p_dl',-0.1013,'n_dl',0.1013,'delta_dl',0);
PBl1.Kj  = struct( 'jt' ,0,'delta_j' ,0,'Ujl' ,pi,'Ljl' ,-pi);
PBl1.Dpl = struct('m_pl',0,'I_pl' ,zeros(3,3),'rcom_pl',zeros(3,1));
PBl1.Ddl = struct('m_dl',3.9,'I_dl' ,diag([42799, 37801, 10997])*1e-6,'rcom_dl',[0;18;-82]*1e-3); 
PBl1.Dj  = struct('Im' ,2.7e-5,'jbv',14.79,'jbc',7.95,'k_tau',0.0429,'k_r',160,'tau_lim',0,'curr_lim',7000,'dq_lim',0,'ddq_lim',0);
PBl2.Mod = struct('ID'  ,1,'typ',1,'Cplx',2,'CANidTX',516,'CANidRX',388);
PBl2.Kpl = struct('a_pl',0,'alpha_pl',0,'p_pl',0,'n_pl',0,'delta_pl',0);
PBl2.Kdl = struct('a_dl',0,'alpha_dl',0,'p_dl',0,'n_dl',0,'delta_dl',0);
PBl2.Kj  = struct( 'jt' ,0,'delta_j' ,0,'Ujl' ,pi,'Ljl' ,-pi);
PBl2.Dpl = struct('m_pl',0,'I_pl' ,zeros(3,3),'rcom_pl',zeros(3,1));
PBl2.Ddl = struct('m_dl',0,'I_dl' ,zeros(3,3),'rcom_dl',zeros(3,1));
PBl2.Dj  = struct('Im' ,2.7e-5,'jbv',15.7,'jbc',7.70,'k_tau',0.0429,'k_r',160,'tau_lim',0,'curr_lim',7000,'dq_lim',0,'ddq_lim',0); % from NewID
PB1 = [PBl1;PBl2];
clear PBl1 PBl2

%init Power Ball 2  
PBl1.Mod = struct('ID'  ,2,'typ',1,'Cplx',2,'CANidTX',517,'CANidRX',389);
PBl1.Kpl = struct('a_pl',0,'alpha_pl',0,'p_pl',0,'n_pl',0,'delta_pl',0);
PBl1.Kdl = struct('a_dl',0,'alpha_dl',-pi/2,'p_dl',-0.1013,'n_dl',0.1013,'delta_dl',0);
PBl1.Kj  = struct( 'jt' ,0,'delta_j' ,0,'Ujl' ,pi,'Ljl' ,-pi);
PBl1.Dpl = struct('m_pl',0,'I_pl' ,zeros(3,3),'rcom_pl',zeros(3,1));
PBl1.Ddl = struct('m_dl',3.9,'I_dl' ,diag([42799, 37801, 10997])*1e-6,'rcom_dl',[0;18;-82]*1e-3); 
PBl1.Dj  = struct('Im' ,2.7e-5,'jbv',18.44,'jbc',7.45,'k_tau',0.0429,'k_r',160,'tau_lim',0,'curr_lim',7000,'dq_lim',0,'ddq_lim',0); % from NewID
PBl2.Mod = struct('ID'  ,2,'typ',1,'Cplx',2,'CANidTX',518,'CANidRX',390);
PBl2.Kpl = struct('a_pl',0,'alpha_pl',0,'p_pl',0,'n_pl',0,'delta_pl',0);
PBl2.Kdl = struct('a_dl',0,'alpha_dl',0,'p_dl',0,'n_dl',0,'delta_dl',0);
PBl2.Kj  = struct( 'jt' ,0,'delta_j' ,0,'Ujl' ,pi,'Ljl' ,-pi);
PBl2.Dpl = struct('m_pl',0,'I_pl' ,zeros(3,3),'rcom_pl',zeros(3,1));
PBl2.Ddl = struct('m_dl',0,'I_dl' ,zeros(3,3),'rcom_dl',zeros(3,1));
PBl2.Dj  = struct('Im' ,2.7e-5,'jbv',16.64,'jbc',6.01,'k_tau',0.0429,'k_r',160,'tau_lim',0,'curr_lim',7000,'dq_lim',0,'ddq_lim',0); % from NewID
PB2 = [PBl1;PBl2];
clear PBl1 PBl2

%init Power Ball 3
PBs1.Mod = struct('ID'  ,3,'typ',1,'Cplx',2,'CANidTX',519,'CANidRX',391);
PBs1.Kpl = struct('a_pl',0,'alpha_pl',0,'p_pl',0,'n_pl',0,'delta_pl',0);
PBs1.Kdl = struct('a_dl',0,'alpha_dl',-pi/2,'p_dl',-0.0748,'n_dl',0.0748,'delta_dl',0);
PBs1.Kj  = struct( 'jt' ,0,'delta_j' ,0,'Ujl' ,pi,'Ljl' ,-pi);
PBs1.Dpl = struct('m_pl',0,'I_pl' ,zeros(3,3),'rcom_pl',zeros(3,1));
PBs1.Ddl = struct('m_dl',1.8,'I_dl' ,diag([12820, 11200, 3200])*1e-6,'rcom_dl',[0;12;-63]*1e-3);
PBs1.Dj  = struct('Im'  ,2.1e-5,'jbv',4.09,'jbc',1.95,'k_tau',0.1,'k_r',100,'tau_lim',0,'curr_lim',2000,'dq_lim',0,'ddq_lim',0); %from newID
PBs2.Mod = struct('ID'  ,3,'typ',1,'Cplx',2,'CANidTX',520,'CANidRX',392);
PBs2.Kpl = struct('a_pl',0,'alpha_pl',0,'p_pl',0,'n_pl',0,'delta_pl',0);
PBs2.Kdl = struct('a_dl',0,'alpha_dl',0,'p_dl',0,'n_dl',0,'delta_dl',0);
PBs2.Kj  = struct( 'jt' ,0,'delta_j' ,0,'Ujl' ,pi,'Ljl' ,-pi);
PBs2.Dpl = struct('m_pl',0,'I_pl' ,zeros(3,3),'rcom_pl',zeros(3,1));
PBs2.Ddl = struct('m_dl',0.1,'I_dl' ,diag([30.7 30.7 33.1])*1e-6,'rcom_dl',[0;0;10]*1e-3); 
PBs2.Dj  = struct('Im'  ,2.1e-5,'jbv',2.96,'jbc',3.30,'k_tau',0.1,'k_r',100,'tau_lim',0,'curr_lim',2000,'dq_lim',0,'ddq_lim',0); %from newID
PB3 = [PBs1;PBs2];
clear PBs1 PBs2

% Link modules
%init Link 1
Ll.Mod = struct('ID'  ,4,'typ',2,'Cplx',1,'CANidTX',0,'CANidRX',0);
Ll.Kpl = struct('a_pl',0.35,'alpha_pl',pi,'p_pl',0,'n_pl',0,'delta_pl',pi/2);
Ll.Kdl = struct('a_dl',0,'alpha_dl',0,'p_dl',0,'n_dl',0,'delta_dl',-pi/2);
Ll.Kj  = struct( 'jt' ,0,'delta_j' ,0,'Ujl' ,0,'Ljl' ,0);
Ll.Dpl = struct('m_pl',1.62,'I_pl' ,[74338 1 -6;1 17132 3742;-6 3742 74338]*1e-6,'rcom_pl',[0; -175; 13.2]*1e-3);
Ll.Ddl = struct('m_dl',0,'I_dl' ,zeros(3,3),'rcom_dl',zeros(3,1));
Ll.Dj  = struct('Im'  ,0,'jbv',0,'jbc',0,'k_tau',0,'k_r',0,'tau_lim',0,'dq_lim',0,'ddq_lim',0);
L1 = Ll;

%init Link 2
Ls.Mod = struct('ID'  ,5,'typ',2,'Cplx',1,'CANidTX',0,'CANidRX',0);
Ls.Kpl = struct('a_pl',0,'alpha_pl',-pi/2,'p_pl',-0.1999,'n_pl',-0.0748,'delta_pl',pi);
Ls.Kdl = struct('a_dl',0,'alpha_dl',0,'p_dl',0,'n_dl',0,'delta_dl',-pi);
Ls.Kj  = struct( 'jt' ,0,'delta_j' ,0,'Ujl' ,0,'Ljl' ,0);
Ls.Dpl = struct('m_pl',1,'I_pl' ,[13909 3 6;3 10894 -5211; 6 -5211 3893]*1e-6,'rcom_pl',[-0.18;47.7;71]*1e-3);
Ls.Ddl = struct('m_dl',0,'I_dl' ,zeros(3,3),'rcom_dl',zeros(3,1));
Ls.Dj  = struct('Im'  ,0,'jbv',0,'jbc',0,'k_tau',0,'k_r',0,'tau_lim',0,'dq_lim',0,'ddq_lim',0);
L2 = Ls;

%init Link 3 (EE)
Lee.Mod = struct('ID'  ,12,'typ',3,'Cplx',1,'CANidTX',0,'CANidRX',0);
Lee.Kpl = struct('a_pl',0,'alpha_pl',0,'p_pl',-0.08,'n_pl',0,'delta_pl',0);
Lee.Kdl = struct('a_dl',0,'alpha_dl',0,'p_dl',0,'n_dl',0,'delta_dl',0);
Lee.Kj  = struct( 'jt' ,0,'delta_j' ,0,'Ujl' ,0,'Ljl' ,0);
Lee.Dpl = struct('m_pl',1.4,'I_pl' ,diag([0.0040 0.0039 0.0006]),'rcom_pl',[0;0;40]*1e-3);
Lee.Ddl = struct('m_dl',0,'I_dl' ,zeros(3,3),'rcom_dl',zeros(3,1));
Lee.Dj  = struct('Im'  ,0,'jbv',0,'jbc',0,'k_tau',0,'k_r',0,'tau_lim',0,'dq_lim',0,'ddq_lim',0);
L3 = Lee;
