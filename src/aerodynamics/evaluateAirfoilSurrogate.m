function out = evaluateAirfoilSurrogate(airfoilDB, foilName, Re, alpha_deg)
% evaluateAirfoilSurrogate
%
% Purpose:
%   Evaluate a prebuilt airfoil surrogate at a requested Reynolds number
%   and optionally at one or more alpha values.
%
% Inputs:
%   airfoilDB   : struct loaded from airfoilDB.mat
%   foilName    : char/string
%   Re          : scalar Reynolds number [-]
%   alpha_deg   : optional scalar/vector [deg]
%
% Outputs:
%   out.name
%   out.Re
%   out.Cla_per_deg
%   out.alphaL0_deg
%   out.Cm0
%   out.Cl_max
%   out.bestLD
%
%   If alpha_deg is provided:
%   out.alpha_deg
%   out.CL
%   out.CD
%   out.CM

    arguments
        airfoilDB struct
        foilName
        Re (1,1) double {mustBePositive}
        alpha_deg double = []
    end

    idx = findAirfoilInDB(airfoilDB, foilName);
    foil = airfoilDB.foils(idx);

    % Reynolds range check
    Re_min = min(airfoilDB.meta.Re_grid);
    Re_max = max(airfoilDB.meta.Re_grid);

    if Re < Re_min || Re > Re_max
        warning('evaluateAirfoilSurrogate:ReOutOfRange', ...
            ['Requested Re = %.3e is outside database range [%.3e, %.3e]. ' ...
             'Nearest extrapolation setting will be used.'], Re, Re_min, Re_max);
    end

    out = struct();
    out.name = foil.name;
    out.Re   = Re;

    out.Cla_per_deg = foil.interp.Cla_per_deg(Re);
    out.alphaL0_deg = foil.interp.alphaL0_deg(Re);
    out.Cm0         = foil.interp.Cm0(Re);
    out.Cl_max      = foil.interp.Cl_max(Re);
    out.bestLD      = foil.interp.bestLD(Re);

    if ~isempty(alpha_deg)
        a = alpha_deg(:);
        r = Re * ones(size(a));

        out.alpha_deg = a;
        out.CL = foil.interp.CL(a, r);
        out.CD = foil.interp.CD(a, r);
        out.CM = foil.interp.CM(a, r);
    end
end