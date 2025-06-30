function [F_thru_sway] = thrusterForce(x, parameters)
   
    U    = parameters.current.vc;
    v    = x(6);
    r    = x(7);

    angle_rad = deg2rad(30);
    Y_portside  = swayForceRudderPropeller(U, v, r, -angle_rad, -angle_rad, parameters);
    Y_starboard = swayForceRudderPropeller(U, v, r, angle_rad, angle_rad, parameters);
    F_thru_sway = [Y_portside, Y_starboard];

end

function Y_R = swayForceRudderPropeller(U, v, r, deltaP, deltaS, parameters)

    R = parameters.rudder;

    % 1) drift angle
    beta = v / U;

    % 2) Wake fraction during maneuvering and inflow velocity at the propeller (Eq. 6 & Eq. 5)
    wP = R.wP0 * exp(-4 * beta^2);
    uP = (1 - wP) * (U + R.yP * r);

    % 3) advance ratio e thrust coefficient (Eq.4)
    JP = uP / (R.nP * R.DP);
    KT = R.k0 + R.k1 * JP + R.k2 * JP^2;

    % 4) Inflow to the rudder and effective angle (Eq. 13, 11, 12, 14)
    uR    = R.eps * uP * sqrt(R.eta*(1 + R.kappa*(sqrt(1 + 8*KT/(pi*JP^2)) - 1))/2 + (1 - R.eta));
    betaR = beta - R.lR * r;
    deltaR= R.gammaR * betaR - atan(R.yR / R.xP);
    vR    = uR * tan(deltaR);

    % 5) Normal force on each rudder (Eq. 9)
    FN_P = 0.5 * parameters.water.rho * R.AR * (uR^2 + vR^2)^(6.13 * R.Lambda/(R.Lambda + 2.25)) * sin(deltaP - deltaR);
    FN_S = 0.5 * parameters.water.rho * R.AR * (uR^2 + vR^2)^(6.13 * R.Lambda/(R.Lambda + 2.25)) * sin(deltaS - deltaR);

    % 6) Sway of the rudders (Eq. 8)
    Y_R = -(1 + R.aH) * (FN_P * cos(deltaP) + FN_S * cos(deltaS));
end
