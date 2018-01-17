function [C_Inf, C_Sup] = sub_Int(A_Inf,A_Sup,B_Inf,B_Sup)
% sub_Int performs the sum between the multidimensional interval A and B:
% [A]-[B] = [C]

C_Inf = A_Inf-B_Sup;
C_Sup = A_Sup-B_Inf;

end

