function [C_Inf, C_Sup] = cross_Int(A_Inf,A_Sup,B_Inf,B_Sup)
% sub_Int performs the sum between the multidimensional interval A and B:
% cross([A],[B]) = [C] for [A], [B], [C] in IR^3

Ax_Inf = A_Inf(1,1);
Ay_Inf = A_Inf(2,1);
Az_Inf = A_Inf(3,1);
Ax_Sup = A_Sup(1,1);
Ay_Sup = A_Sup(2,1);
Az_Sup = A_Sup(3,1);
Bx_Inf = B_Inf(1,1);
By_Inf = B_Inf(2,1);
Bz_Inf = B_Inf(3,1);
Bx_Sup = B_Sup(1,1);
By_Sup = B_Sup(2,1);
Bz_Sup = B_Sup(3,1);


[p1_Inf, p1_Sup]  = prod_Int(Ay_Inf,Ay_Sup,Bz_Inf,Bz_Sup);% Ay*Bz
[p2_Inf, p2_Sup]  = prod_Int(Az_Inf,Az_Sup,By_Inf,By_Sup);% Az*By
[Cx_Inf, Cx_Sup]  = sub_Int(p1_Inf,p1_Sup,p2_Inf,p2_Sup);% Ay*Bz-Az*By;

[p3_Inf, p3_Sup]  = prod_Int(Az_Inf,Az_Sup,Bx_Inf,Bx_Sup);% Az*Bx
[p4_Inf, p4_Sup]  = prod_Int(Ax_Inf,Ax_Sup,Bz_Inf,Bz_Sup);% Ax*Bz
[Cy_Inf, Cy_Sup]  = sub_Int(p3_Inf,p3_Sup,p4_Inf,p4_Sup);% Az*Bx-Ax*Bz;

[p5_Inf, p5_Sup]  = prod_Int(Ax_Inf,Ax_Sup,By_Inf,By_Sup);% Ax*By
[p6_Inf, p6_Sup]  = prod_Int(Ay_Inf,Ay_Sup,Bx_Inf,Bx_Sup);% Ay*Bx
[Cz_Inf, Cz_Sup]  = sub_Int(p5_Inf,p5_Sup,p6_Inf,p6_Sup);% Ax*By-Ay*Bx;

C_Inf = [Cx_Inf; Cy_Inf; Cz_Inf];
C_Sup = [Cx_Sup; Cy_Sup; Cz_Sup];

end

