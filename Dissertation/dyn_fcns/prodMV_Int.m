function [C_Inf, C_Sup] = prodMV_Int(A_Inf,A_Sup,B_Inf,B_Sup)
% sub_Int performs the sum between the multidimensional interval A and B:
% [A].*[B] = [C]
% 
% if size(A_Inf) > 1 || size(A_Sup) > 1 || size(B_Inf) > 1 || size(B_Sup) > 1   
%     error('Dimensions of input variables not supported.');
% end

% A Matrix 3x3 B vector
if size(A_Inf,2) == 3 && size(A_Inf,1) == 3 && size(B_Inf,2) == 1 && size(B_Inf,1) == 3
    A11_Inf = A_Inf(1,1);
    A12_Inf = A_Inf(1,2);
    A13_Inf = A_Inf(1,3);
    A21_Inf = A_Inf(2,1);
    A22_Inf = A_Inf(2,2);
    A23_Inf = A_Inf(2,3);
    A31_Inf = A_Inf(3,1);
    A32_Inf = A_Inf(3,2);
    A33_Inf = A_Inf(3,3);
    A11_Sup = A_Sup(1,1);
    A12_Sup = A_Sup(1,2);
    A13_Sup = A_Sup(1,3);
    A21_Sup = A_Sup(2,1);
    A22_Sup = A_Sup(2,2);
    A23_Sup = A_Sup(2,3);
    A31_Sup = A_Sup(3,1);
    A32_Sup = A_Sup(3,2);
    A33_Sup = A_Sup(3,3);
    
    Bx_Inf = B_Inf(1,1);
    By_Inf = B_Inf(2,1);
    Bz_Inf = B_Inf(3,1);
    Bx_Sup = B_Sup(1,1);
    By_Sup = B_Sup(2,1);
    Bz_Sup = B_Sup(3,1);
    
    %Cx = A11*Bx + A12*By + A13*Bz;
    %      -p1-     -p2-     -p3-
    %           -s1-
    [p1_Inf, p1_Sup] = prod_Int(A11_Inf,A11_Sup,Bx_Inf,Bx_Sup);
    [p2_Inf, p2_Sup] = prod_Int(A12_Inf,A12_Sup,By_Inf,By_Sup);
    [p3_Inf, p3_Sup] = prod_Int(A13_Inf,A13_Sup,Bz_Inf,Bz_Sup);
    [s1_Inf, s1_Sup] = sum_Int(p1_Inf, p1_Sup, p2_Inf, p2_Sup);
    [Cx_Inf, Cx_Sup] = sum_Int(s1_Inf, s1_Sup, p3_Inf, p3_Sup);
    
    %Cy = A21*Bx + A22*By + A23*Bz;
    %      -p4-     -p5-     -p6-
    %           -s2-
    [p4_Inf, p4_Sup] = prod_Int(A21_Inf,A21_Sup,Bx_Inf,Bx_Sup);
    [p5_Inf, p5_Sup] = prod_Int(A22_Inf,A22_Sup,By_Inf,By_Sup);
    [p6_Inf, p6_Sup] = prod_Int(A23_Inf,A23_Sup,Bz_Inf,Bz_Sup);
    [s2_Inf, s2_Sup] = sum_Int(p4_Inf, p4_Sup, p5_Inf, p5_Sup);
    [Cy_Inf, Cy_Sup] = sum_Int(s2_Inf, s2_Sup, p6_Inf, p6_Sup);
    
    %Cz = A31*Bx + A32*By + A33*Bz;
    %      -p7-     -p8-     -p9-
    %           -s3-
    [p7_Inf, p7_Sup] = prod_Int(A31_Inf,A31_Sup,Bx_Inf,Bx_Sup);
    [p8_Inf, p8_Sup] = prod_Int(A32_Inf,A32_Sup,By_Inf,By_Sup);
    [p9_Inf, p9_Sup] = prod_Int(A33_Inf,A33_Sup,Bz_Inf,Bz_Sup);
    [s3_Inf, s3_Sup] = sum_Int(p7_Inf, p7_Sup, p8_Inf, p8_Sup);
    [Cz_Inf, Cz_Sup] = sum_Int(s3_Inf, s3_Sup, p9_Inf, p9_Sup);
    
    C_Inf = [Cx_Inf; Cy_Inf; Cz_Inf];
    C_Sup = [Cx_Sup; Cy_Sup; Cz_Sup];
end

end

