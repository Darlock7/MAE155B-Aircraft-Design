function foilOut = runSingleAirfoilXFOIL(foilName, Re, alpha_deg, opts)
% runSingleAirfoilXFOIL
%
% Purpose:
%   Thin wrapper around existing airfoilAnalysisXFOIL(...) so that one foil
%   can be run at one Reynolds number using the already-developed parser.
%
% Inputs:
%   foilName    : char/string, e.g. 'e222.dat' or 'NACA4415'
%   Re          : scalar Reynolds number [-]
%   alpha_deg   : vector of angle of attack values [deg]
%   opts        : struct with fields:
%                   .xfoilFolder
%                   .Mach
%                   .maxIter
%                   .cleanupFiles
%                   .printSummary   (optional)
%                   .airfoilFolder  (optional)
%
% Output:
%   foilOut     : struct containing one-airfoil polar + derived metrics
%
% Notes:
%   This wrapper duplicates the same foil into both the root and tip slots
%   and returns the "root" result.

    arguments
        foilName
        Re (1,1) double {mustBePositive}
        alpha_deg (:,1) double
        opts struct
    end

    if ~isfield(opts, 'xfoilFolder') || isempty(opts.xfoilFolder)
        opts.xfoilFolder = '.';
    end
    if ~isfield(opts, 'Mach') || isempty(opts.Mach)
        opts.Mach = 0.0;
    end
    if ~isfield(opts, 'maxIter') || isempty(opts.maxIter)
        opts.maxIter = 150;
    end
    if ~isfield(opts, 'cleanupFiles') || isempty(opts.cleanupFiles)
        opts.cleanupFiles = true;
    end
    if ~isfield(opts, 'printSummary') || isempty(opts.printSummary)
        opts.printSummary = false;
    end
    if ~isfield(opts, 'airfoilFolder') || isempty(opts.airfoilFolder)
        opts.airfoilFolder = '.';
    end

    foilName = char(string(foilName));

    % If this is a coordinate file, make sure XFOIL can see it from the cwd.
    isNACA = startsWith(upper(strtrim(foilName)), 'NACA');

    copiedLocal = false;
    srcFile = fullfile(opts.airfoilFolder, foilName);
    dstFile = fullfile(pwd, foilName);

    if ~isNACA
        if exist(foilName, 'file') ~= 2
            if exist(srcFile, 'file') == 2
                copyfile(srcFile, dstFile);
                copiedLocal = true;
            else
                error('runSingleAirfoilXFOIL:FileNotFound', ...
                    'Could not find airfoil file "%s" in cwd or airfoilFolder.', foilName);
            end
        end
    end

    airfoilIn = struct();
    airfoilIn.rootFoil = foilName;
    airfoilIn.tipFoil  = foilName;

    airfoilIn.Re_root = Re;
    airfoilIn.Re_tip  = Re;

    airfoilIn.alpha_deg    = alpha_deg(:).';
    airfoilIn.xfoilFolder  = opts.xfoilFolder;
    airfoilIn.Mach         = opts.Mach;
    airfoilIn.maxIter      = opts.maxIter;
    airfoilIn.cleanupFiles = opts.cleanupFiles;
    airfoilIn.printSummary = opts.printSummary;

    tmp = airfoilAnalysisXFOIL(airfoilIn);

    if ~isfield(tmp, 'root')
        error('runSingleAirfoilXFOIL:MissingRoot', ...
            'Expected output field "root" was not found.');
    end

    foilOut = tmp.root;

    if copiedLocal && exist(dstFile, 'file') == 2
        delete(dstFile);
    end
end