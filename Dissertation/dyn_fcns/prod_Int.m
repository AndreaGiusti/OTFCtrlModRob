function [C_Inf, C_Sup] = prod_Int(A_Inf,A_Sup,B_Inf,B_Sup)
% sub_Int performs the sum between the multidimensional interval A and B:
% [A].*[B] = [C]
% 
% if size(A_Inf) > 1 || size(A_Sup) > 1 || size(B_Inf) > 1 || size(B_Sup) > 1   
%     error('Dimensions of input variables not supported.');
% end

% scalar case
if max([size(A_Inf,1),size(A_Sup,1),size(B_Inf,1),size(B_Sup,1)]) == 1
    C_Inf = min([A_Inf.*B_Inf A_Inf.*B_Sup A_Sup.*B_Inf A_Sup.*B_Sup]);
    C_Sup = max([A_Inf.*B_Inf A_Inf.*B_Sup A_Sup.*B_Inf A_Sup.*B_Sup]);
end

end

