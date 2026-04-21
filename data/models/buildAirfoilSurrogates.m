%% buildAirfoilSurrogates.m
% Purpose:
%   Offline script to build Reynolds-based surrogate models for multiple airfoils
%   using the existing airfoilAnalysisXFOIL(...) function.
%
% Usage:
%   Run this script once whenever you add airfoils or want to refresh the
%   surrogate database.
%
% Output:
%   Saves:
%       data/airfoilSurrogates/airfoilDB.mat
%
% Notes:
%   - This script is intentionally standalone.
%   - The main sizing code should NOT call XFOIL directly after this is built.
%   - Chord effects are handled indirectly through Reynolds number:
%         Re = rho * V * c / mu

clc; clearvars; close all;

fprintf('=============================================================\n');
fprintf('Building airfoil surrogate database from XFOIL\n');
fprintf('=============================================================\n');

repoRoot = fileparts(mfilename('fullpath'));

%% ---------------- User settings ----------------

% Airfoils to include
foilList = { ...
    'mh60.dat', ...
    'mh61.dat', ...
    'mh62.dat', ...
    'mh64.dat', ...
    'e222.dat', ...
    'e230.dat', ...
    'e387.dat', ...
    'eh3012.dat', ...
    'naca4415.dat'};

% Alpha grid for stored polars
alpha_deg = (-6:0.5:12).';     % [deg]

% Reynolds-number grid for surrogate generation
Re_grid = [ ...
    5e4 ...
    7.5e4 ...
    1e5 ...
    1.5e5 ...
    2e5 ...
    3e5 ...
    4e5 ...
    5e5 ...
    7.5e5 ...
    1e6 ];

% XFOIL settings
opts = struct();
opts.xfoilFolder  = repoRoot;    % <-- change if XFOIL exe lives elsewhere
opts.Mach         = 0.03;        % [-]
opts.maxIter      = 150;         % [-]
opts.cleanupFiles = true;        % delete temp files
opts.printSummary = false;       % keep console cleaner

% Folder for airfoil coordinate files if not in repo root
opts.airfoilFolder = repoRoot;   % <-- change if needed

% Output location
outputFolder = fullfile(repoRoot, 'data', 'airfoilSurrogates');
outputFile   = fullfile(outputFolder, 'airfoilDB.mat');

if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end

%% ---------------- Init database ----------------

airfoilDB = struct();
airfoilDB.meta = struct();
airfoilDB.meta.alpha_deg = alpha_deg;
airfoilDB.meta.Re_grid   = Re_grid(:).';
airfoilDB.meta.Mach      = opts.Mach;
airfoilDB.meta.maxIter   = opts.maxIter;
airfoilDB.meta.source    = 'Generated from XFOIL using buildAirfoilSurrogates.m';
airfoilDB.meta.created   = char(datetime('now'));
airfoilDB.foils = struct([]);

%% ---------------- Build each airfoil ----------------

for iFoil = 1:numel(foilList)

    foilName = foilList{iFoil};
    fprintf('\n-------------------------------------------------------------\n');
    fprintf('Airfoil %d / %d : %s\n', iFoil, numel(foilList), foilName);
    fprintf('-------------------------------------------------------------\n');

    nAlpha = numel(alpha_deg);
    nRe    = numel(Re_grid);

    CL = nan(nAlpha, nRe);
    CD = nan(nAlpha, nRe);
    CM = nan(nAlpha, nRe);

    Cla_per_deg = nan(1, nRe);
    alphaL0_deg = nan(1, nRe);
    Cm0         = nan(1, nRe);
    Cl_max      = nan(1, nRe);
    bestLD      = nan(1, nRe);

    alpha_valid_min_deg = nan(1, nRe);
    alpha_valid_max_deg = nan(1, nRe);
    nPolarPoints        = zeros(1, nRe);

    validRe = false(1, nRe);

    for j = 1:nRe
        Re = Re_grid(j);
        fprintf('  Re = %.3e ... ', Re);

        try
            out = runSingleAirfoilXFOIL(foilName, Re, alpha_deg, opts);

            % ---- Required field checks ----
            requiredPolarFields = {'alpha_deg','CL','CD','CM'};
            requiredMetricFields = {'Cla_per_deg','alphaL0_deg','Cm0','Cl_max','bestLD'};

            for k = 1:numel(requiredPolarFields)
                assert(isfield(out, requiredPolarFields{k}), ...
                    'Missing field "%s" in XFOIL output for %s at Re=%.3e.', ...
                    requiredPolarFields{k}, foilName, Re);
            end

            for k = 1:numel(requiredMetricFields)
                assert(isfield(out, requiredMetricFields{k}), ...
                    'Missing field "%s" in XFOIL output for %s at Re=%.3e.', ...
                    requiredMetricFields{k}, foilName, Re);
            end

            alpha_out = out.alpha_deg(:);
            CL_out    = out.CL(:);
            CD_out    = out.CD(:);
            CM_out    = out.CM(:);

            % Map returned XFOIL points onto the master alpha grid
            [tf, loc] = ismembertol(alpha_deg, alpha_out, 1e-10, 'DataScale', 1);

            CL(tf, j) = CL_out(loc(tf));
            CD(tf, j) = CD_out(loc(tf));
            CM(tf, j) = CM_out(loc(tf));

            Cla_per_deg(j) = out.Cla_per_deg;
            alphaL0_deg(j) = out.alphaL0_deg;
            Cm0(j)         = out.Cm0;
            Cl_max(j)      = out.Cl_max;
            bestLD(j)      = out.bestLD;

            alpha_valid_min_deg(j) = min(alpha_out);
            alpha_valid_max_deg(j) = max(alpha_out);
            nPolarPoints(j)        = numel(alpha_out);

            validRe(j) = true;
            fprintf('ok (%d pts)\n', numel(alpha_out));

        catch ME
            fprintf('FAILED\n');
            warning('buildAirfoilSurrogates:XFOILFail', ...
                'Foil %s at Re = %.3e failed.\nReason: %s', foilName, Re, ME.message);
        end
    end

    % ---- Fill gaps across Reynolds direction only ----
    % Do not fill an entire column if it is completely empty.
    for iAlpha = 1:nAlpha
        rowCL = CL(iAlpha, :);
        rowCD = CD(iAlpha, :);
        rowCM = CM(iAlpha, :);

        if any(isfinite(rowCL))
            CL(iAlpha, :) = fillmissing(rowCL, 'linear', 'EndValues', 'nearest');
        end
        if any(isfinite(rowCD))
            CD(iAlpha, :) = fillmissing(rowCD, 'linear', 'EndValues', 'nearest');
        end
        if any(isfinite(rowCM))
            CM(iAlpha, :) = fillmissing(rowCM, 'linear', 'EndValues', 'nearest');
        end
    end

    if any(isfinite(Cla_per_deg))
        Cla_per_deg = fillmissing(Cla_per_deg, 'linear', 'EndValues', 'nearest');
    end
    if any(isfinite(alphaL0_deg))
        alphaL0_deg = fillmissing(alphaL0_deg, 'linear', 'EndValues', 'nearest');
    end
    if any(isfinite(Cm0))
        Cm0 = fillmissing(Cm0, 'linear', 'EndValues', 'nearest');
    end
    if any(isfinite(Cl_max))
        Cl_max = fillmissing(Cl_max, 'linear', 'EndValues', 'nearest');
    end
    if any(isfinite(bestLD))
        bestLD = fillmissing(bestLD, 'linear', 'EndValues', 'nearest');
    end

    % ---- Sanity checks ----
    if ~any(validRe)
        error('No valid Reynolds-number runs were obtained for airfoil %s.', foilName);
    end

    % ---- Build interpolants ----
    [AA, RR] = ndgrid(alpha_deg, Re_grid);

    foil = struct();
    foil.name = foilName;
    foil.validRe = validRe;

    foil.alpha_valid_min_deg = alpha_valid_min_deg;
    foil.alpha_valid_max_deg = alpha_valid_max_deg;
    foil.nPolarPoints        = nPolarPoints;

    foil.polar = struct();
    foil.polar.alpha_deg = alpha_deg;
    foil.polar.Re_grid   = Re_grid;
    foil.polar.CL = CL;
    foil.polar.CD = CD;
    foil.polar.CM = CM;

    foil.metrics = struct();
    foil.metrics.Cla_per_deg = Cla_per_deg;
    foil.metrics.alphaL0_deg = alphaL0_deg;
    foil.metrics.Cm0         = Cm0;
    foil.metrics.Cl_max      = Cl_max;
    foil.metrics.bestLD      = bestLD;

    foil.interp = struct();
    foil.interp.CL = griddedInterpolant(AA, RR, CL, 'linear', 'nearest');
    foil.interp.CD = griddedInterpolant(AA, RR, CD, 'linear', 'nearest');
    foil.interp.CM = griddedInterpolant(AA, RR, CM, 'linear', 'nearest');

    foil.interp.Cla_per_deg = griddedInterpolant(Re_grid, Cla_per_deg, 'linear', 'nearest');
    foil.interp.alphaL0_deg = griddedInterpolant(Re_grid, alphaL0_deg, 'linear', 'nearest');
    foil.interp.Cm0         = griddedInterpolant(Re_grid, Cm0, 'linear', 'nearest');
    foil.interp.Cl_max      = griddedInterpolant(Re_grid, Cl_max, 'linear', 'nearest');
    foil.interp.bestLD      = griddedInterpolant(Re_grid, bestLD, 'linear', 'nearest');

    airfoilDB.foils(iFoil) = foil; %#ok<SAGROW>
end

%% ---------------- Save database ----------------

save(outputFile, 'airfoilDB', '-v7.3');

fprintf('\n=============================================================\n');
fprintf('Airfoil surrogate database saved to:\n%s\n', outputFile);
fprintf('=============================================================\n');