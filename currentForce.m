function [F_curr_sway] = currentForce(x, parameters)

psi_deg = x(3);
u_ship  = x(5);
v_ship  = x(6);

Vc      = parameters.current.vc;
alphac  = parameters.current.alphac;

L       = parameters.shuttle.length;
T       = parameters.shuttle.draft;
S       = parameters.shuttle.s;
B       = parameters.shuttle.beam;
Cy      = parameters.shuttle.cy;
Cb      = parameters.shuttle.cb;
lp      = parameters.shuttle.lp;
rho     = parameters.water.rho;
mu      = parameters.water.mu;

alphac = alphac + 180;
alpha_diff = alphac - psi_deg;

u_rel = u_ship - Vc * cosd(alpha_diff);
v_rel = v_ship - Vc * sind(alpha_diff);

Vcr = sqrt(u_rel^2 + v_rel^2);
alphar = atan2(v_rel, u_rel);

Re = rho * Vc * L / mu;
% Re = 2.5 * 10^5;

% C0 = 0.09375 / (log10(Re) - 2)^2 * S / (T * L);
% C1C = C0 * cos(alphar) + (pi/8) * (T/L) * (cos(3*alphar) - cos(alphar));

C2C = (Cy - (pi*T)/(2*L)) * sin(alphar) * abs(sin(alphar)) ...
    + (pi*T)/(2*L) * (sin(alphar)^3) ...
    + ((pi*T)/L) * (1 + 0.4*(Cb*B/T)) * sin(alphar) * abs(cos(alphar));

% C6C = ...
%     - (lp / L) * Cy * sin(alphar) * abs(sin(alphar)) ...
%     - (pi * T / L) * sin(alphar) * cos(alphar) ...
%     - ( (1 + abs(cos(alphar))) / 2)^2 * (pi * T / L) ...
%     * (0.5 - 2.4 * (T / L)) * sin(alphar) * abs(cos(alphar));

%F_curr_surge = 0.5 * rho * Vcr^2 * L * T * C1C;
F_curr_sway  = 0.5 * rho * Vcr^2 * L * T * C2C;
%M_curr_yaw   = 0.5 * rho * Vcr^2 * L^2 * T * C6C;

end
