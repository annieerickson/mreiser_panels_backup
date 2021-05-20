%MAKE_PATS_RT_3PANELSHIGH.M Generates patterns for MReiser's LED panels: 
% Some stripe patterns; astral projections: rotating / translating;
% Saves as .mat files.
% based on code by Peter Weir.
% ivo.ros@gmail.com

% clear all
close all
% hardcode
NUMVERTPIX = 24;
NUMHORIZPIX = 96;
NUM_GS_BITS = 4; % number of greenscale bits to use
LVL = 2; % brightness of the panels
ARENA_DEGPERPIX = 360/NUMHORIZPIX; 
PIXFOR360 = 360/ARENA_DEGPERPIX;
FRAMERATE_FRAMESPERSEC = 60;
MOTIONDURATION_SEC = PIXFOR360 / FRAMERATE_FRAMESPERSEC;                      % motion duration needs to be 
% softcode
pattern.x_num = MOTIONDURATION_SEC * FRAMERATE_FRAMESPERSEC;                % There are 96 pixel around the display (12x8), but virtually, there are 160, for a 360 deg surround. Times 3 = 480, because at least 4 seconds are needed in each direction
pattern.y_num = 12;
pattern.num_panels = (NUMVERTPIX/8)*(NUMHORIZPIX/8);                                                    % This is the number of unique Panel IDs required.
% pattern.gs_val = 2;                                                         % This pattern will use 3 intensity levels: ‘2’ indicates all pixel values in pattern.Pats are either 0, 1, 2, or 3.
pattern.gs_val = NUM_GS_BITS;                                                         % This pattern will use 16 intensity levels: ‘4’ indicates number of characters for binary code
pattern.row_compression = 0;                                                % hardcoded line-compressed patterns
Pats = ones(NUMVERTPIX, NUMHORIZPIX, pattern.x_num, pattern.y_num)*LVL;
%% 1 all off
Pats(:,:,:,1) = zeros(NUMVERTPIX, NUMHORIZPIX, pattern.x_num);
%% 11: stripe for CL mode.
k=11;
stripe=6;
Pats(:, (PIXFOR360/2)-(stripe/2)+1-4:(PIXFOR360/2)+(stripe/2)-4, 1, k) = repmat(zeros(1,stripe)*LVL, NUMVERTPIX, 1);
for j = 2:pattern.x_num % for close-loop stripe fixation to work, pattern.x_num needs to be an integer factor times PIXFOR360
    Pats(:,:,j,k) = ShiftMatrix(Pats(:,:,j-1,k),1,'r','y');
end
%% 3: prey
k=3;
frames = ones(NUMVERTPIX, NUMHORIZPIX, pattern.x_num)*LVL;
Pats(:,:,:,k) = frames;
Pats(9,(PIXFOR360/2)-4,1,k) = 0; % 1 pixel sun
% horizontal shift
for j = 2:pattern.x_num % for close-loop stripe fixation to work, pattern.x_num needs to be an integer factor times PIXFOR360
    Pats(:,:,j,k) = ShiftMatrix(Pats(:,:,j-1,k),1,'r','y');
end

% %% 3: sun for CL mode
% k=3;
% frames = zeros(NUMVERTPIX, NUMHORIZPIX, pattern.x_num);
% Pats(:,:,:,k) = frames;
% % Pats(12:13,(PIXFOR360/2)-4:(PIXFOR360/2)+1-4 ,1,k) = ones(2,2)*7; % 2x2 pixel sun
% Pats(9,(PIXFOR360/2)-4,1,k) = 7; % 1 pixel sun
% % horizontal shift
% for j = 2:pattern.x_num % for close-loop stripe fixation to work, pattern.x_num needs to be an integer factor times PIXFOR360
%     Pats(:,:,j,k) = ShiftMatrix(Pats(:,:,j-1,k),1,'r','y');
% end

%% Rotations, 4:6
%% Define universe of points and constants
DENSITYOFPOINTS_PERCUBICMETER = 20;
MAXSENSORYRADIUS_METERS = 2.0;
% motion:
XVELOCITY_METERSPERSEC = 2.0;
% XVELlow = 3.0;
% XVELhigh = 5.0;
YVELOCITY_METERSPERSEC = 0;
ZVELOCITY_METERSPERSEC = 0;
ROTAXIS_ROLLp=[1 0 0];
ROTAXIS_PITCHp=[0 1 0];
ROTAXIS_YAWp=[0 0 1];
ROTMAG_DEGPERFRAME = ARENA_DEGPERPIX;
ARENAINCLINATIONCENTER_PIX = (NUMVERTPIX+1)/2;
ARENAAZIMUTHCENTER_PIX = (NUMHORIZPIX+1)/2-4;
% constants:
DEGPERRAD = 180/pi;
%% Calculate arena coordinates
rotrate_degps = ROTMAG_DEGPERFRAME * FRAMERATE_FRAMESPERSEC; % 2.25 * 60 = 120 deg/s
arenaCircumference_pix = 360.0/ARENA_DEGPERPIX;
arenaRadius_pix = arenaCircumference_pix/(2*pi);
arenaAzimuthCoordinates_deg = ([1:NUMHORIZPIX] - ARENAAZIMUTHCENTER_PIX)*ARENA_DEGPERPIX;
arenaAzimuthCoordinates_rad = arenaAzimuthCoordinates_deg/DEGPERRAD;
arenaInclinationCoordinates_rad = pi/2 - atan(([1:NUMVERTPIX] - ARENAINCLINATIONCENTER_PIX)/arenaRadius_pix);
%% Populate space with uniformly randomly distributed points
speed_metersPerSec = sqrt(XVELOCITY_METERSPERSEC^2 + YVELOCITY_METERSPERSEC^2 + ZVELOCITY_METERSPERSEC^2);
maxDistMeters = MAXSENSORYRADIUS_METERS + MOTIONDURATION_SEC*speed_metersPerSec;
numPoints = round(DENSITYOFPOINTS_PERCUBICMETER*(2*maxDistMeters)^3);       % keeping density constant irrespective of size
xyz_meters = (rand(numPoints,3)*2 - 1)*maxDistMeters;                       % from normalized around 0 (max +/- 1 along all axes), to absolute covering max dist
%% Create a time base
dt=1/FRAMERATE_FRAMESPERSEC; % 1/40
t_sec = 0:dt:MOTIONDURATION_SEC-dt;
%% 4: Calculate frames ROTAXIS=ROTAXIS_ROLLp;
rotMagFact = 2;
ROTAXIS=ROTAXIS_ROLLp;
frames = ones(NUMVERTPIX, NUMHORIZPIX, pattern.x_num)*LVL;
for frameInd = 1:pattern.x_num
    tThisFrame_sec = t_sec(frameInd);
    rotmag=rotrate_degps*rotMagFact*tThisFrame_sec;
    xyzThisFrame_meters = quatrot(xyz_meters,rotmag,ROTAXIS);
    distanceThisFrame_meters =  sqrt(sum(xyzThisFrame_meters.^2,2));
    inclinationThisFrame_rad = acos(xyzThisFrame_meters(:,3)./distanceThisFrame_meters);
    azimuthThisFrame_rad = atan2(xyzThisFrame_meters(:,2),xyzThisFrame_meters(:,1));
    for pointIndex = 1:numPoints
        distanceThisPoint_meters = distanceThisFrame_meters(pointIndex);
        inclinationThisPoint_rad = inclinationThisFrame_rad(pointIndex);
        azimuthThisPoint_rad = azimuthThisFrame_rad(pointIndex);
        isInSensoryRange = (distanceThisPoint_meters <= MAXSENSORYRADIUS_METERS);
        isInInclinationRange = ((inclinationThisPoint_rad >= min(arenaInclinationCoordinates_rad)) & (inclinationThisPoint_rad <= max(arenaInclinationCoordinates_rad)));
        isInAzimuthRange = ((azimuthThisPoint_rad >= min(arenaAzimuthCoordinates_rad)) & (azimuthThisPoint_rad <= max(arenaAzimuthCoordinates_rad)));
        if isInSensoryRange && isInInclinationRange && isInAzimuthRange
            [minE, inclinationCoordinate] = min(abs(inclinationThisPoint_rad - arenaInclinationCoordinates_rad));
            [minA, azimuthCoordinate] = min(abs(azimuthThisPoint_rad - arenaAzimuthCoordinates_rad));
            frames(inclinationCoordinate,azimuthCoordinate,frameInd) = 0;
        end
    end
% 	imwrite(frames(:,:,frameInd),sprintf('Roll %d .png', frameInd))
%      imshow(frames(:,:,frameInd))
%      drawnow
end
Pats(:,:,:,4) = frames;
%% 5: ROTAXIS=ROTAXIS_PITCHp;
rotMagFact = 1.5;
ROTAXIS=ROTAXIS_PITCHp;
frames = ones(NUMVERTPIX, NUMHORIZPIX, pattern.x_num)*LVL;
for frameInd = 1:pattern.x_num
    tThisFrame_sec = t_sec(frameInd);
    rotmag=rotrate_degps*rotMagFact*tThisFrame_sec;
    xyzThisFrame_meters = quatrot(xyz_meters,rotmag,ROTAXIS);
    distanceThisFrame_meters =  sqrt(sum(xyzThisFrame_meters.^2,2));
    inclinationThisFrame_rad = acos(xyzThisFrame_meters(:,3)./distanceThisFrame_meters);
    azimuthThisFrame_rad = atan2(xyzThisFrame_meters(:,2),xyzThisFrame_meters(:,1));
    for pointIndex = 1:numPoints
        distanceThisPoint_meters = distanceThisFrame_meters(pointIndex);
        inclinationThisPoint_rad = inclinationThisFrame_rad(pointIndex);
        azimuthThisPoint_rad = azimuthThisFrame_rad(pointIndex);
        isInSensoryRange = (distanceThisPoint_meters <= MAXSENSORYRADIUS_METERS);
        isInInclinationRange = ((inclinationThisPoint_rad >= min(arenaInclinationCoordinates_rad)) & (inclinationThisPoint_rad <= max(arenaInclinationCoordinates_rad)));
        isInAzimuthRange = ((azimuthThisPoint_rad >= min(arenaAzimuthCoordinates_rad)) & (azimuthThisPoint_rad <= max(arenaAzimuthCoordinates_rad)));
        if isInSensoryRange && isInInclinationRange && isInAzimuthRange
            [minE, inclinationCoordinate] = min(abs(inclinationThisPoint_rad - arenaInclinationCoordinates_rad));
            [minA, azimuthCoordinate] = min(abs(azimuthThisPoint_rad - arenaAzimuthCoordinates_rad));
            frames(inclinationCoordinate,azimuthCoordinate,frameInd) = 0;
        end
    end
end
Pats(:,:,:,5) = frames;
%% 6: ROTAXIS=ROTAXIS_YAWp;
rotMagFact = 1;
ROTAXIS=-ROTAXIS_YAWp; % to rotate correct way (positive is yaw motion to the right)
frames = ones(NUMVERTPIX, NUMHORIZPIX, pattern.x_num)*LVL;
for frameInd = 1:pattern.x_num
    tThisFrame_sec = t_sec(frameInd);
    rotmag=rotrate_degps*rotMagFact*tThisFrame_sec;
    xyzThisFrame_meters = quatrot(xyz_meters,rotmag,ROTAXIS);
    distanceThisFrame_meters =  sqrt(sum(xyzThisFrame_meters.^2,2));
    inclinationThisFrame_rad = acos(xyzThisFrame_meters(:,3)./distanceThisFrame_meters);
    azimuthThisFrame_rad = atan2(xyzThisFrame_meters(:,2),xyzThisFrame_meters(:,1));
    for pointIndex = 1:numPoints
        distanceThisPoint_meters = distanceThisFrame_meters(pointIndex);
        inclinationThisPoint_rad = inclinationThisFrame_rad(pointIndex);
        azimuthThisPoint_rad = azimuthThisFrame_rad(pointIndex);
        isInSensoryRange = (distanceThisPoint_meters <= MAXSENSORYRADIUS_METERS);
        isInInclinationRange = ((inclinationThisPoint_rad >= min(arenaInclinationCoordinates_rad)) & (inclinationThisPoint_rad <= max(arenaInclinationCoordinates_rad)));
        isInAzimuthRange = ((azimuthThisPoint_rad >= min(arenaAzimuthCoordinates_rad)) & (azimuthThisPoint_rad <= max(arenaAzimuthCoordinates_rad)));
        if isInSensoryRange && isInInclinationRange && isInAzimuthRange
            [minE, inclinationCoordinate] = min(abs(inclinationThisPoint_rad - arenaInclinationCoordinates_rad));
            [minA, azimuthCoordinate] = min(abs(azimuthThisPoint_rad - arenaAzimuthCoordinates_rad));
            frames(inclinationCoordinate,azimuthCoordinate,frameInd) = 0;
        end
    end
end
Pats(:,:,:,6) = frames;
%% Translation. 7
%% 7: 0 deg. From N (Progressive)
VELOCITY_METERSPERSEC = [XVELOCITY_METERSPERSEC 0 0];
frames = ones(NUMVERTPIX, NUMHORIZPIX, pattern.x_num)*LVL;
for frameInd = 1:pattern.x_num
    tThisFrame_sec = t_sec(frameInd);
    xyzThisFrame_meters = xyz_meters - ones(numPoints,1)*VELOCITY_METERSPERSEC*tThisFrame_sec;
    distanceThisFrame_meters =  sqrt(sum(xyzThisFrame_meters.^2,2));
    inclinationThisFrame_rad = acos(xyzThisFrame_meters(:,3)./distanceThisFrame_meters);
    azimuthThisFrame_rad = atan2(xyzThisFrame_meters(:,2),xyzThisFrame_meters(:,1));
    for pointIndex = 1:numPoints
        distanceThisPoint_meters = distanceThisFrame_meters(pointIndex);
        inclinationThisPoint_rad = inclinationThisFrame_rad(pointIndex);
        azimuthThisPoint_rad = azimuthThisFrame_rad(pointIndex);
        isInSensoryRange = (distanceThisPoint_meters <= MAXSENSORYRADIUS_METERS);
        isInInclinationRange = ((inclinationThisPoint_rad >= min(arenaInclinationCoordinates_rad)) & (inclinationThisPoint_rad <= max(arenaInclinationCoordinates_rad)));
        isInAzimuthRange = ((azimuthThisPoint_rad >= min(arenaAzimuthCoordinates_rad)) & (azimuthThisPoint_rad <= max(arenaAzimuthCoordinates_rad)));
        if isInSensoryRange && isInInclinationRange && isInAzimuthRange
            [minE, inclinationCoordinate] = min(abs(inclinationThisPoint_rad - arenaInclinationCoordinates_rad));
            [minA, azimuthCoordinate] = min(abs(azimuthThisPoint_rad - arenaAzimuthCoordinates_rad));
            frames(inclinationCoordinate,azimuthCoordinate,frameInd) = 0;
        end
    end
end
Pats(:,:,:,7) = frames;
%% invert 
% for subpat = 1:7
%     Pats(:,:,:,subpat) = round(LVL-Pats(:,:,:,subpat));
% end
%% Prepping for PControl.m, by putting remainder in language of patterns
disp('Prepping for PControl')
pattern.Pats = Pats;
pattern.Panel_map = [12 8 4 11 7 3 10 6 2 9 5 1; 24 20 16 23 19 15 22 18 14 21 17 13; 36 32 28 35 31 27 34 30 26 33 29 25;];
pattern.BitMapIndex = process_panel_map(pattern);
% Commenting out the following line will allow you to view the pattern in the pattern player before taking the time to make them for writing to the SD card 
pattern.data = Make_pattern_vector(pattern);
% size (Pats)
thisFullFileName =  mfilename('fullpath');
[directory_name,thisFileName,thisFileExtension] = fileparts(thisFullFileName);
str = [directory_name '\Pattern_' thisFileName(6:end)];
save(str, 'pattern');
disp(['Saved to: ' str])