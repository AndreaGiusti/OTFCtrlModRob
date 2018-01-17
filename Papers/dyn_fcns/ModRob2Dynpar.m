% ModRob2Dynpar - Computed the sinthesized link parameters of an assembled
%                 robot described by ModRob
%
% Syntax:  
%    ModRob2Dynpar(ModRob,DH_ext)
%
% Inputs: 
%    ModRob -> ordered array of the module structures with respect to 
%              the robot assembly
%    DH_ext -> Extanded DH table of parameters, the i-th row is 
%              [a_i, alpha_i, gamma_i, d_i, p_i, n_i, jt_i, phi_i],
%              (refer to [1] for further details on the parameters).
% Outputs:
%    Dyn_par_out -> array of structures collecting dyn. pars of each robot
%                   link 
%
% Other m-files required: Tr(), Ro(), Skew(), proto_module()
% Subfunctions: none
% MAT-files required: none
%
% See also: 

% Author:       Andrea Giusti
% Written:      14-12-2015
% Last update:  05-03-2017
% Last revision:---

%------------- BEGIN CODE --------------

function [Dyn_par_out] = ModRob2Dynpar(ModRob,DH_ext)
%#codegen

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

% init output variable
pm = proto_module();
Dj = pm.Dj;

Dyn_par = repmat(struct('m',0,'I',zeros(3,3),'DJ',Dj,'r_com',zeros(3,1)), length(ModRob), 1 );
Dyn_par_out = Dyn_par;

% Skip all possible initial link modules
h = 1;
while(ModRob(h).Mod.typ == 2 && h <= N)
    h=h+1;
end

% core procedure
cntJ = 0;
k = h;
while k <= (N-1)
    cntJ = cntJ+1;
    
    m_dl_im1     = ModRob(k).Ddl.m_dl;
    I_dl_im1     = ModRob(k).Ddl.I_dl;
    r_dl_com_im1 = ModRob(k).Ddl.rcom_dl;
    DJ_im1       = ModRob(k).Dj;
    
    f = k+1;
    while (ModRob(f).Mod.typ == 2 && f < N)
        m_pl_i     = ModRob(f).Dpl.m_pl;
        I_pl_i     = ModRob(f).Dpl.I_pl;
        r_pl_com_i = ModRob(f).Dpl.rcom_pl;
        m_i        = m_dl_im1+m_pl_i;
        I_io_i     = I_dl_im1+I_pl_i;
        r_io_com_i = (m_dl_im1*r_dl_com_im1+m_pl_i*r_pl_com_i)/m_i;
        % Transform the vectors with respect to the D-H frame
        % center of mass
        p_l         = ModRob(f).Kpl.p_pl;
        a_l         = ModRob(f).Kpl.a_pl;
        n_l         = ModRob(f).Kpl.n_pl;
        alpha_l     = ModRob(f).Kpl.alpha_pl;
        delta_l_in  = ModRob(f).Kpl.delta_pl;
        delta_l_out = ModRob(f).Kdl.delta_dl;
        Aio_i = Ro('z',-delta_l_in)*Tr('z',-p_l)*Tr('x',a_l)*...
            Ro('x',alpha_l)*Tr('z',n_l)*Ro('z',delta_l_out);
        U_aux = pinv(Aio_i)*[r_io_com_i;1];
        r_dl_com_im1 = U_aux(1:3);
        % inertia tensor
        R_aux = Aio_i(1:3,1:3);
        I_dl_im1 = transpose(R_aux)*(I_io_i-m_i*transpose(Skew(r_io_com_i))*Skew(r_io_com_i))*R_aux+m_i*transpose(Skew(U_aux(1:3)))*Skew(U_aux(1:3));
        m_dl_im1 = m_i; 
        
        f=f+1;
    end
    k = f-1;
    
    m_pl_i     = ModRob(k+1).Dpl.m_pl;
    I_pl_i     = ModRob(k+1).Dpl.I_pl;
    r_pl_com_i = ModRob(k+1).Dpl.rcom_pl;
    p_pl       = ModRob(k+1).Kpl.p_pl;
    a_pl       = ModRob(k+1).Kpl.a_pl;
    n_pl       = ModRob(k+1).Kpl.n_pl;
    alpha_pl   = ModRob(k+1).Kpl.alpha_pl;
    delta_pl   = ModRob(k+1).Kpl.delta_pl;
    phi_i      = DH_ext(cntJ+1,8);
    
    m_i = m_dl_im1+m_pl_i;
    I_io_i       = I_dl_im1+I_pl_i;
    r_io_com_i   = (m_dl_im1*r_dl_com_im1+m_pl_i*r_pl_com_i)/m_i;
    
    Dyn_par(cntJ).m = m_i;
    Dyn_par(cntJ).DJ = DJ_im1;
    % Transform the vectors with respect to the D-H frame
    % center of mass
    Aio_i = Ro('z',-delta_pl)*Tr('z',-p_pl)*Tr('x',a_pl)*...
            Ro('x',alpha_pl)*Tr('z',n_pl)*Ro('z',phi_i);
    U_aux = pinv(Aio_i)*[r_io_com_i;1];
    Dyn_par(cntJ).r_com = U_aux(1:3);
    % inertia tensor
    R_aux = Aio_i(1:3,1:3); 
    I_aux = transpose(R_aux)*(I_io_i-m_i*transpose(Skew(r_io_com_i))*Skew(r_io_com_i))*R_aux+m_i*transpose(Skew(U_aux(1:3)))*Skew(U_aux(1:3));
    Dyn_par(cntJ).I = I_aux;
    
    k=k+1;
end
if(N>0)
    if ModRob(N).Mod.typ == 3 || ModRob(N).Mod.typ == 2
    %The last synthesis
        m_dl_im1     = ModRob(N-1).Ddl.m_dl;
        I_dl_im1     = ModRob(N-1).Ddl.I_dl;
        r_dl_com_im1 = ModRob(N-1).Ddl.rcom_dl;
        DJ_im1       = ModRob(N-1).Dj;
        m_pl_i     = ModRob(N).Dpl.m_pl;
        I_pl_i     = ModRob(N).Dpl.I_pl;
        r_pl_com_i = ModRob(N).Dpl.rcom_pl;
        m_i        = m_dl_im1+m_pl_i;
        I_io_i     = I_dl_im1+I_pl_i;
        r_io_com_i = (m_dl_im1*r_dl_com_im1+m_pl_i*r_pl_com_i)/m_i;
        p_l         = ModRob(N).Kpl.p_pl;
        a_l         = ModRob(N).Kpl.a_pl;
        n_l         = ModRob(N).Kpl.n_pl;
        alpha_l     = ModRob(N).Kpl.alpha_pl;
        delta_l_in  = ModRob(N).Kpl.delta_pl;
        delta_l_out = ModRob(N).Kdl.delta_dl;
        Aio_i = Ro('z',-delta_l_in)*Tr('z',-p_l)*Tr('x',a_l)*...
            Ro('x',alpha_l)*Tr('z',n_l)*Ro('z',delta_l_out);
        
        Dyn_par(NJ).m = m_i;
        Dyn_par(NJ).DJ = DJ_im1;
        
        U_aux = pinv(Aio_i)*[r_io_com_i;1];
        Dyn_par(NJ).r_com = U_aux(1:3);
        % inertia tensor
        R_aux = Aio_i(1:3,1:3); 
        I_aux = transpose(R_aux)*(I_io_i-m_i*transpose(Skew(r_io_com_i))*Skew(r_io_com_i))*R_aux+m_i*transpose(Skew(U_aux(1:3)))*Skew(U_aux(1:3));
        Dyn_par(NJ).I = I_aux;
        Dyn_par_out(1:NJ) = Dyn_par(1:NJ);
    else
    %The last link is the last distal
        %friction
        Dyn_par(NJ).DJ = ModRob(N).Dj;
        % mass
        Dyn_par(NJ).m = ModRob(N).Ddl.m_dl;
        % center of mass, we only need to rotate to align the frame to the D-H
        % one
        A_aux = Ro('z',-ModRob(N).Kpl.delta_pl);
        U_aux = pinv(A_aux)*[ModRob(N).Ddl.rcom_dl;1];
        Dyn_par(NJ).r_com = U_aux(1:3);
        % inertia tensor
        R_aux = A_aux(1:3,1:3);
        I_aux = transpose(R_aux)*ModRob(N).Ddl.I_dl*R_aux;
        Dyn_par(NJ).I = I_aux;
        Dyn_par_out(1:NJ) = Dyn_par(1:NJ);
    end
else
    Dyn_par_out = Dyn_par;
end
    
end
