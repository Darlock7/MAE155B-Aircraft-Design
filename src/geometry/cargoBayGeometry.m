function cargoOut = cargoBayGeometry(cargoIn)
% cargoBayGeometry
%
% Purpose:
%   Calculates the maximum rectangular cargo bay cross-section that fits
%   inside a NACA 0012 airfoil shape. The rectangle is oriented with sides
%   parallel to the chord line (horizontal) and perpendicular to it (vertical).
%
% Inputs:
%   cargoIn.L_fuse_m       fuselage length [m]
%   cargoIn.airfoilFile    path to airfoil coordinate file (default: n0012.dat)
%
% Outputs:
%   cargoOut.width_m       maximum rectangle width (chord direction) [m]
%   cargoOut.height_m      maximum rectangle height (perpendicular to chord) [m]
%   cargoOut.area_m2       rectangular cross-sectional area [m^2]
%   cargoOut.volume_m3     cargo bay volume (width * height * length) [m^3]
%   cargoOut.x_coords      rectangle x-coords [4x1]
%   cargoOut.y_coords      rectangle y-coords [4x1]
%   cargoOut.airfoil_x     airfoil x-coordinates
%   cargoOut.airfoil_y     airfoil y-coordinates

    arguments
        cargoIn struct
    end

    % Default airfoil file
    if ~isfield(cargoIn, 'airfoilFile')
        % Try to find the airfoil file in typical locations
        candidates = {
            'data/airfoils/n0012.dat',
            '..\data\airfoils\n0012.dat',
            '..\..\data\airfoils\n0012.dat'
        };
        found = false;
        for i = 1:numel(candidates)
            if exist(candidates{i}, 'file')
                cargoIn.airfoilFile = candidates{i};
                found = true;
                break;
            end
        end
        if ~found
            error('cargoBayGeometry:AirfoilFileNotFound', ...
                'Could not find n0012.dat airfoil file');
        end
    end

    L_fuse_m = cargoIn.L_fuse_m;

    % Load airfoil coordinates from file
    fid = fopen(cargoIn.airfoilFile, 'r');
    if fid == -1
        error('cargoBayGeometry:FileOpenError', ...
            'Cannot open airfoil file: %s', cargoIn.airfoilFile);
    end
    
    % Skip header lines (first 3 lines based on typical .dat format)
    fgetl(fid);
    fgetl(fid);
    fgetl(fid);
    
    % Read coordinates
    coords = fscanf(fid, '%f %f', [2 inf]);
    fclose(fid);
    
    airfoil_x = coords(1, :);
    airfoil_y = coords(2, :);

    % Airfoil coordinates are typically normalized to chord = 1
    x_norm = airfoil_x;
    y_norm = airfoil_y;
    
    % Find max thickness and its location
    [max_thickness, idx_max] = max(abs(y_norm));
    x_max_thick = x_norm(idx_max);
    
    % The airfoil is symmetric (or nearly so), so we work with upper surface
    
    % Extract upper surface (y >= 0)
    upper_idx = y_norm >= 0;
    x_upper = x_norm(upper_idx);
    y_upper = y_norm(upper_idx);
    
    % Find the maximum rectangle that fits inside the airfoil
    % For a symmetric airfoil, the maximum rectangle will be centered
    % We need to find the maximum width at each height, then find optimal height
    
    % Search over possible heights (as fraction of max thickness)
    n_search = 1000;
    max_area = 0;
    optimal_height = 0;
    optimal_width = 0;
    
    for i = 1:n_search
        h_frac = i / n_search;  % Height as fraction of max thickness
        h = h_frac * max_thickness;
        
        % Find the x-range where airfoil is above this height
        % For upper surface: y >= h
        valid_idx = y_upper >= h;
        
        if any(valid_idx)
            x_min = min(x_upper(valid_idx));
            x_max = max(x_upper(valid_idx));
            width = x_max - x_min;
            
            area = width * h * 2;  % *2 for both upper and lower (symmetric)
            
            if area > max_area
                max_area = area;
                optimal_height = h;
                optimal_width = width;
            end
        end
    end
    
    % Convert back to actual dimensions (scale by chord = length for fuselage cross-section)
    % The fuselage cross-section uses the airfoil shape, so chord = length
    chord_m = L_fuse_m;
    
    cargoOut.width_m = optimal_width * chord_m;
    cargoOut.height_m = optimal_height * 2 * chord_m;  % Full height (upper + lower)
    cargoOut.area_m2 = cargoOut.width_m * cargoOut.height_m;
    cargoOut.volume_m3 = cargoOut.area_m2 * L_fuse_m;
    
    % Rectangle coordinates (centered at airfoil center)
    x_center = (min(x_upper(y_upper >= optimal_height)) + max(x_upper(y_upper >= optimal_height))) / 2;
    
    % Scaled rectangle corners
    x_rect = [x_center - optimal_width/2, x_center + optimal_width/2, ...
              x_center + optimal_width/2, x_center - optimal_width/2] * chord_m;
    y_rect = [-optimal_height, -optimal_height, optimal_height, optimal_height] * chord_m;
    
    cargoOut.x_coords = x_rect;
    cargoOut.y_coords = y_rect;
    
    % Airfoil coordinates scaled to actual size
    cargoOut.airfoil_x = airfoil_x * chord_m;
    cargoOut.airfoil_y = airfoil_y * chord_m;
    
    % Also return normalized values for reference
    cargoOut.width_norm = optimal_width;
    cargoOut.height_norm = optimal_height * 2;
    cargoOut.max_thickness_norm = max_thickness;
    
    % Plot if requested
    if isfield(cargoIn, 'showPlot') && cargoIn.showPlot
        figure('Name', 'Cargo Bay Cross-Section');
        hold on;
        
        % Plot airfoil
        fill(cargoOut.airfoil_x, cargoOut.airfoil_y, [0.8 0.9 1], ...
            'EdgeColor', 'b', 'LineWidth', 1.5);
        
        % Plot rectangle
        fill(cargoOut.x_coords, cargoOut.y_coords, [1 0.8 0.8], ...
            'EdgeColor', 'r', 'LineWidth', 2);
        
        % Plot rectangle outline
        plot(cargoOut.x_coords, cargoOut.y_coords, 'r-', 'LineWidth', 2);
        plot([cargoOut.x_coords(1), cargoOut.x_coords(4)], ...
             [cargoOut.y_coords(1), cargoOut.y_coords(4)], 'r-', 'LineWidth', 2);
        
        % Add dimension annotations
        % Width arrow
        y_annot = cargoOut.height_m/2 + 0.01;
        annotation('arrow', [0.3 0.7], [0.8 0.8], 'Color', 'k', 'LineWidth', 1.5);
        
        xlabel('X (m) - Chord Direction');
        ylabel('Y (m) - Thickness Direction');
        
        % Extract airfoil name from filename for title
        [~, name, ~] = fileparts(cargoIn.airfoilFile);
        name = upper(name);
        title(sprintf('%s Cargo Bay: %.3f m x %.3f m = %.4f m²', ...
            name, cargoOut.width_m, cargoOut.height_m, cargoOut.area_m2));
        axis equal;
        grid on;
        legend('Airfoil', 'Max Rectangle', 'Location', 'best');
        
        % Add text box with dimensions
        text_str = sprintf('Width: %.3f m\nHeight: %.3f m\nArea: %.4f m²\nVolume: %.5f m³', ...
            cargoOut.width_m, cargoOut.height_m, cargoOut.area_m2, cargoOut.volume_m3);
        annotation('textbox', [0.15 0.7 0.2 0.15], 'String', text_str, ...
            'FitBoxToText', 'on', 'BackgroundColor', 'white', 'EdgeColor', 'k');
    end
end