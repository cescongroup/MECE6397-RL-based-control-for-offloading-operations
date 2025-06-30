function [F_iner_sway] = inertiaForce(x, parameters)

M       = parameters.shuttle.m;
M11     = parameters.shuttle.m11;
M22     = parameters.shuttle.m22;
M66     = parameters.shuttle.m66;
M26     = parameters.shuttle.m26;
xG      = parameters.shuttle.lp;
Iz      = parameters.shuttle.iz;
lBow    = parameters.shuttle.lBow;
Lh      = parameters.hawser.length;
L       = parameters.shuttle.length;

A11 = (M + M11);
A22 = (M + M22);
A26 = (M * xG + M26);
A66 = (Iz + M66);

A = A22*Lh/L;
B = -A22*lBow/(L+A26);
C = A11-A22;

F_iner_sway = A * x(9) + B * x(10) + C * x(8);
%disp(F_iner_sway);

end
