function animate(parameters, Time, x, Vc, alphac)

% animate – Displays an animation of the shuttle-tanker movement while it is
%           connected to the FPSO, including current vectors across the
%           simulation area.
%
% Parameters:
%   parameters : structure containing sizing and positioning data for the FPSO
%                and the shuttle tanker
%   Time       : vector of simulation time instants corresponding to each frame
%   x          : matrix with columns [CGmoveX, CGmoveY, CGangle] describing:
%                – X displacement
%                – Y displacement
%                – angle increment (or absolute angle) in degrees
%   simOut     : structure holding the simulation results, in particular the
%                current data
%                simOut.Current{1}.Values → timeseries object with:
%                    Data(:,1) = current magnitude
%                    Data(:,2) = current direction (degrees)
%
% Requirements:
%   The images 'shuttle_tanker_image.png' and 'fpso_image.png' must be located
%   in the same directory.


    %% Extracts data from the shuttle tanker and the FPSO
    CGangle = x(:,3);
    CGmoveX = x(:,1);
    CGmoveY = x(:,2);

   %% Loads and scales the images
    % Shuttle tanker
    shuttle_img = imread('shuttle_tanker_image.png');
    [orig_h, orig_w, ~] = size(shuttle_img);
    scale_shuttle = parameters.shuttle.length / orig_w;
    
    % FPSO
    fpso_img = imread('fpso_image.png');
    [fpso_h, fpso_w, ~] = size(fpso_img);
    scale_fpso = parameters.fpso.length / fpso_w;
    
    %% Definition of the Shuttle’s Rotation Point (CG)
    CGx = (1-parameters.shuttle.cgx) * orig_w;
    CGy = parameters.shuttle.cgy * orig_h;
    
    % Reference point for the shuttle’s bow (if you want to mark the bow)
    shuttle_bow_x = 2*parameters.shuttle.length;
    shuttle_bow_y = 0.5 * orig_h;
    
    %% Configures the canvas (output coordinate system)
    canvas_size = ceil(sqrt(orig_h^2 + orig_w^2));
    outputView = imref2d([canvas_size, canvas_size], [-700 500], [-400 400]);
   
    
    %% Sets up the transformation and displays the FPSO
    T_fpso = [ scale_fpso, 0,          0;
               0,           scale_fpso, 0;
               parameters.fpso.BowX0 - scale_fpso, ...
               parameters.fpso.BowY0 - scale_fpso*((fpso_h+1)/2), ...
               1 ];
    tform_fpso = affine2d(T_fpso);
    fpso_img_warped = imwarp(fpso_img, tform_fpso, 'OutputView', outputView, 'FillValues', 255);
    
    figure;
    h_fpso = imshow(fpso_img_warped, outputView);
    hold on;
    
    % Marks the fixed point on the FPSO (e.g., right side)
    x_right  = parameters.fpso.BowX0;
    y_center = parameters.fpso.BowY0;
    plot(x_right, y_center, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 1);
  
    %% Reading and plotting the current vectors
    % Assuming simOut.Current{1}.Values is a timeseries with:
    % Data(:,1) = magnitude, Data(:,2) = angle (degrees)
    %tsCurrent = simOut.Current{1}.Values;

    
    % If you want to use an “average” value or just the first value:
    magnitudeCurrent = Vc;
    angleCurrent     = alphac;
    
    % Converts to components (u, v)
    u = magnitudeCurrent * cosd(angleCurrent);
    v = magnitudeCurrent * sind(angleCurrent);
    
    % Creates a grid of points from -100 to 1200 (X) and -300 to 300 (Y), in steps of 100
    [xGrid, yGrid] = meshgrid(-700:100:500, -400:100:400);
    
    % Plots the current vectors (no autoscaling → scale factor 0)
    quiver(xGrid, yGrid, u*ones(size(xGrid)), v*ones(size(yGrid)), ...
           0.3, 'Color', 'k', 'LineWidth', 0.2);
    
    %% Initial position of the shuttle
    blueX = parameters.shuttle.CGX0;
    blueY = parameters.shuttle.CGY0;
    
    % Initial angle
    cumulativeAngle = CGangle(1);
    angle = cumulativeAngle;
    
    %% Initial display of the shuttle
    T_shuttle = [ scale_shuttle*cosd(angle),  scale_shuttle*sind(angle), 0;
                 -scale_shuttle*sind(angle), scale_shuttle*cosd(angle), 0;
                  blueX - scale_shuttle*cosd(angle)*CGx + scale_shuttle*sind(angle)*CGy, ...
                  blueY - scale_shuttle*sind(angle)*CGx - scale_shuttle*cosd(angle)*CGy, 1 ];
    tform_shuttle = affine2d(T_shuttle);
    rotated_img = imwarp(shuttle_img, tform_shuttle, 'OutputView', outputView, 'FillValues', 255);
    
    % Creates a transparency mask (any white region becomes transparent)
    if size(rotated_img,3) == 3
        mask = (rotated_img(:,:,1)==255 & rotated_img(:,:,2)==255 & rotated_img(:,:,3)==255);
    else
        mask = (rotated_img==255);
    end
    alphaData = ones(size(mask));
    alphaData(mask) = 0;
    
    % Displays the shuttle image
    h_shuttle = imshow(rotated_img, outputView);
    set(h_shuttle, 'AlphaData', alphaData);
    set(gca, 'YDir', 'normal');
    title(['Angle: ' num2str(angle) '°, Time: ' num2str(Time(1)) ' s']);
    
    % Markers (CG and bow) and line (hawser)
    h_blue = plot(blueX, blueY, 'wo', 'MarkerSize', 4, 'MarkerFaceColor', 'w');
    [bowX, bowY] = transformPointsForward(tform_shuttle, shuttle_bow_x, shuttle_bow_y);
    h_red = plot(bowX, bowY, 'bo', 'MarkerSize', 2, 'MarkerFaceColor', 'b');
    h_line = plot([x_right, bowX], [y_center, bowY], 'k-', 'LineWidth', 1);
    
    drawnow;
   
    %% Selects the indices to update the animation every 10 s (for example)
    tFinal = max(Time);
    tDesejados = 0:1:tFinal;  % a cada 10 segundos
    indices = zeros(1, length(tDesejados));
    for k = 1:length(tDesejados)
        [~, idxMin] = min(abs(Time - tDesejados(k)));
        indices(k) = idxMin;
    end
    
    %% Animation loop
    for idx = indices
        % Updates the angle
        cumulativeAngle = CGangle(idx);
        angle = cumulativeAngle;

        % Sets up the shuttle transformation
        T_shuttle = [ scale_shuttle*cosd(angle),  scale_shuttle*sind(angle), 0;
                     -scale_shuttle*sind(angle), scale_shuttle*cosd(angle), 0;
                      blueX - scale_shuttle*cosd(angle)*CGx + scale_shuttle*sind(angle)*CGy, ...
                      blueY - scale_shuttle*sind(angle)*CGx - scale_shuttle*cosd(angle)*CGy, 1 ];
        tform_shuttle = affine2d(T_shuttle);
        rotated_img = imwarp(shuttle_img, tform_shuttle, 'OutputView', outputView, 'FillValues', 255);
        
        % Updates the transparency mask
        if size(rotated_img,3) == 3
            mask = (rotated_img(:,:,1)==255 & rotated_img(:,:,2)==255 & rotated_img(:,:,3)==255);
        else
            mask = (rotated_img==255);
        end
        alphaData = ones(size(mask));
        alphaData(mask) = 0;
        
        % Updates the shuttle image
        set(h_shuttle, 'CData', rotated_img, 'AlphaData', alphaData);
        title(['Angle: ' num2str(angle) '°, Time: ' num2str(Time(idx)) ' s']);
        
        % Updates the markers and the line (hawser)
        set(h_blue, 'XData', blueX, 'YData', blueY);
        [bowX, bowY] = transformPointsForward(tform_shuttle, shuttle_bow_x, shuttle_bow_y);
        set(h_red, 'XData', bowX, 'YData', bowY);
        set(h_line, 'XData', [x_right, bowX], 'YData', [y_center, bowY]);

       
        drawnow;
        % % If you want to pause between each frame, uncomment:
        % pause(0.1);
        
        % % Updates the shuttle’s position (displacements)
        %blueX = parameters.shuttle.CGX0 + CGmoveX(idx)
        %blueY = parameters.shuttle.CGY0 + CGmoveY(idx)
        blueX =  CGmoveX(idx);
        blueY =  CGmoveY(idx);

    end
end
