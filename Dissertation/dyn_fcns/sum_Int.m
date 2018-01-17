function [C_Inf, C_Sup] = sum_Int(A_Inf,A_Sup,B_Inf,B_Sup)
% sum_Int performs the sum between the multidimensional interval A and B:
% [A]+[B] =[C]

C_Inf = A_Inf+B_Inf;
C_Sup = A_Sup+B_Sup;

end

