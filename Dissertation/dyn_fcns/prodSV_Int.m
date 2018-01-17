function [C_Inf, C_Sup] = prodSV_Int(A_Inf,A_Sup,B_Inf,B_Sup)
% sub_Int performs the sum between the multidimensional interval A and B:
% [A].*[B] = [C]
% 
% if size(A_Inf) > 1 || size(A_Sup) > 1 || size(B_Inf) > 1 || size(B_Sup) > 1   
%     error('Dimensions of input variables not supported.');
% end

% A scalar B vector
if max([size(A_Inf,1),size(A_Sup,1)]) == 1 && max([size(B_Inf,1),size(B_Sup,1)]) > 1 && size(A_Sup,2) == 1
    Bx_Inf = B_Inf(1,1);
    By_Inf = B_Inf(2,1);
    Bz_Inf = B_Inf(3,1);
    Bx_Sup = B_Sup(1,1);
    By_Sup = B_Sup(2,1);
    Bz_Sup = B_Sup(3,1);
    
    Cx_Inf = min([A_Inf.*Bx_Inf A_Inf.*Bx_Sup A_Sup.*Bx_Inf A_Sup.*Bx_Sup]);
    Cx_Sup = max([A_Inf.*Bx_Inf A_Inf.*Bx_Sup A_Sup.*Bx_Inf A_Sup.*Bx_Sup]);
    Cy_Inf = min([A_Inf.*By_Inf A_Inf.*By_Sup A_Sup.*By_Inf A_Sup.*By_Sup]);
    Cy_Sup = max([A_Inf.*By_Inf A_Inf.*By_Sup A_Sup.*By_Inf A_Sup.*By_Sup]);
    Cz_Inf = min([A_Inf.*Bz_Inf A_Inf.*Bz_Sup A_Sup.*Bz_Inf A_Sup.*Bz_Sup]);
    Cz_Sup = max([A_Inf.*Bz_Inf A_Inf.*Bz_Sup A_Sup.*Bz_Inf A_Sup.*Bz_Sup]);
    
    C_Inf = [Cx_Inf; Cy_Inf; Cz_Inf];
    C_Sup = [Cx_Sup; Cy_Sup; Cz_Sup];
end

end

