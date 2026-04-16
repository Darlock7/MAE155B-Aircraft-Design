function J = profitPerUnitTime(Wp, Vp, Ef, Wg, Tf)
% ============================================================
% MAE 155B Spring 2026 Scoring Function
%
% Inputs:
%   Wp [N]   = package 1 weight
%   Vp [m^3] = package 2 volume
%   Ef [J]   = energy consumption (already includes x20 factor)
%   Wg [N]   = gross weight
%   Tf [s]   = flight time (already includes x20 factor)
%
% Output:
%   J [$ / s] = score (profit per unit time)
% ============================================================

    % --- Parameters (from scoring slide) ---
    rWp = 2.25e-1;     % Revenue per Unit Payload Weight [$/N]
    rVp = 3.00e2;      % Revenue per Unit Payload Volume [$/m^3]
    ce  = 1.39e-6;     % Cost per Unit Energy [$/J]
    cc  = 5.00e-1;     % Cost per Flight [$]
    cWg = 2.25e1;      % Cost per Unit Gross Weight[$/N]
    cf  = 1.11e-4;     % Cost per Unit Flight Time [$/s]
    tl  = 1.44e6;      % Total Flight Time in Aircraft Life [s]

    % --- Score calculation ---
    J = (rWp*Wp + rVp*Vp - ce*Ef - cc)/Tf ...
        - (cWg*Wg)/tl ...
        - cf;

end