% ModRob2DH_f -  Returns the full D-H table using information in ModRob
%
% Syntax:  
%    ModRob2DH_f(ModRob)
%
% Inputs: 
%    ModRob -> Ordered array of the module structures with respect to 
%              the robot assembly 
%
% Outputs:
%    DHtab -> Standard DH table, the i-th row of DHtab is
%             [a_i, alpha_i, gamma_i, d_i].
%    jt    -> Id vector of joint types (0 for rev., 1 for prism.)
%    B     -> B is the hom. transf. matrix from the basis to the 
%             first joint
%   jlimit -> each row is [Ljl,Ujl] of the corresponding joint
%
%   NJ     -> number of joint modules in ModRob
%
% NB: for now ModRob has to end with a joint module!!!
%
% Example: 
%
% Other m-files required: ModRob2DHext(ModRob), DHext2DH(DHext).
% Subfunctions: none
% MAT-files required: none
%
% See also: 
%
% Author:       Andrea Giusti
% Written:      14-December-2015
% Last update:  ---
% Last revision:---

%------------- BEGIN CODE --------------

function [DHtab,jt,B,jlimit,NJ] = ModRob2DH_f(ModRob)

jlimit = zeros(length(ModRob),2);
% count number of joint modules in ModRob
NJ = 0;
for k =1:length(ModRob)
    if (ModRob(k).Mod.ID ~= 0 && ModRob(k).Mod.typ == 1)
        NJ = NJ+1;
        jlimit(NJ,:) = [ModRob(k).Kj.Ljl, ModRob(k).Kj.Ujl];
    end
end

% Extract DHtab and jt of proper dimension
[DHext,B] = ModRob2DHext(ModRob);
[DHtab, jt] = DHext2DH(DHext);

end
