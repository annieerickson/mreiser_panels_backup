%MAKE_PATS_PP_SMPL.M Generates patterns for MReiser's LED panels: 
% Some stripe patterns; astral projections: rotating / translating; an
% expansion pattern, and oscillating wide field patterns

% Generate here and save as .mat files.
% 8-pixel wide stripes
% astral projections, 0
% roll+ (play with negative gain for reverse)
% pitch
% yaw

% based on code from P.T. Weir
% 05 2019 ivo.ros@gmail.com

% clear all
close all
%% Prep
% softcode the following?
NUMVERTPIX      = 32;
NUMPIXAZIM      = 96;
ARENA_DEGPERPIX = 216/NUMPIXAZIM; 
PIXFOR360       = 360/ARENA_DEGPERPIX; %160
% MOTIONDURATION_SEC = 4;
FRAMERATE_FRAMESPERSEC = 60;
MOTIONDURATION_SEC = PIXFOR360 / FRAMERATE_FRAMESPERSEC;                    % motion duration needs to be 
pattern.x_num = MOTIONDURATION_SEC * FRAMERATE_FRAMESPERSEC;                % There are 96 pixel around the display (12x8), but virtually, there are 160, for a 360 deg surround. Times 3 = 480, because at least 4 seconds are needed in each direction
pattern.y_num = 13;
pattern.num_panels = 48;                                                    % This is the number of unique Panel IDs required.
% pattern.gs_val = 2;                                                         % This pattern will use 3 intensity levels: ‘2’ indicates all pixel values in pattern.Pats are either 0, 1, 2, or 3.
pattern.gs_val = 3;                                                         % This pattern will use 7 intensity levels: ‘3’ indicates all pixel values in pattern.Pats are either 0, 1, 2, 3, 4, 5, 6, or 7 (000, 001, 010, 011, 100, 101, 110, 111).
pattern.row_compression = 0;                                                % hardcoded line-compressed patterns
%LVL = 1;
LVL = 7;
Pats = zeros(NUMVERTPIX, PIXFOR360, pattern.x_num, pattern.y_num);
%% 1, stripe, and 3, sun, for CL mode.
% 6 pixel stripe; 
stripe=6;
Pats(:, (PIXFOR360/2)-(stripe/2)+1:(PIXFOR360/2)+(stripe/2), 1, 1) = repmat(ones(1,stripe)*LVL, NUMVERTPIX, 1);
% for a polar opposite stripe too.
%Pats(:, (PIXFOR360)-(stripe/2)+1:end, 1, 1)                        = repmat(ones(1,stripe/2), NUMVERTPIX, 1);
%Pats(:, 1:stripe/2, 1, 1)                                          = repmat(ones(1,stripe/2), NUMVERTPIX, 1);

% 8 pixel stripe; Square-wave grating: spat. freq. 36 degs/cycle
% stripe=8;
% numHorCycs=PIXFOR360/(stripe*2);
% Pats(:, :, 1, 11) = repmat([zeros(1,stripe) ones(1,stripe)*LVL], NUMVERTPIX, numHorCycs);

%% 3, 2x2 pixel sun
Pats(12:13,(PIXFOR360/2):(PIXFOR360/2)+1      ,1,3) = ones(2,2)*7;
% horizontal shift
for j = 2:pattern.x_num % for close-loop stripe fixation to work, pattern.x_num needs to be an integer factor times PIXFOR360
    Pats(:,:,j,1) = ShiftMatrix(Pats(:,:,j-1,1),1,'r','y');
    Pats(:,:,j,3) = ShiftMatrix(Pats(:,:,j-1,3),1,'r','y');
%     Pats(:,:,j,3) = ShiftMatrix(Pats(:,:,j-1,3),1,'r','y');
%     Pats(:,:,j,4) = ShiftMatrix(Pats(:,:,j-1,4),1,'r','y');
end
%% 11 Oscillating 2x2 pixel sun. sinusoidally at .5Hz.
% 2x2 pixel sun
k = 11;
negSevtFvDeg_pix = ceil((PIXFOR360/12)*3.5); % 26
posSevtFvDeg_pix = ceil((PIXFOR360/12)*8.5);
binNum = 40; % half the spatial wavelength in pixels. At 40 Hz pattern refresh rate a binNum of 40 will oscillate the stripe at .5Hz.
Pats(12:13,posSevtFvDeg_pix:posSevtFvDeg_pix+1 ,1,k) = ones(2,2)*7;
Pats(12:13,negSevtFvDeg_pix-1:negSevtFvDeg_pix ,binNum+1,k) = ones(2,2)*7;

binDist=zeros(binNum+1,3);
for bin = 1:binNum
    binDist(bin+1,1) = binDist(bin,1) + sin((bin)/(binNum+1)*pi)*1/(binNum+1); % binNum+1. Integral of a sine wave
end
binDistMax = binDist((binNum+1),1);
binDist(:,1) = binDist(:,1)/binDistMax * (posSevtFvDeg_pix-negSevtFvDeg_pix+1); % pixel location relative to lowest, offset pixel

for bin = 1:binNum
    binDist(bin,2:3) = [floor(binDist(bin,1)) binDist(bin,1)- floor(binDist(bin,1))]; % [integer number-integer] (integers and fractions)
end
% plot(binDist(:,1))
% assume 2 pixels are at 1 and 1 (intensity) when binDist is at integer.
for bin = 2:binNum
    revbin = binNum + 2 - bin; % from binNum (40) to 2, [dt to 1-dt] % hardcode 1 at frame 1
    revbinRvrs = (binNum + 2 - revbin) + binNum; % from binNum + 1 (42) to 80 [1+dt to 2-dt] % hardcode 1 at frame 80
    pixInt = binDist(bin,2);
    pixFrac = binDist(bin,3);
    LEDon = ones(2,3).*[1-pixFrac 1 pixFrac];
    Pats(12:13, negSevtFvDeg_pix+pixInt-1:negSevtFvDeg_pix+pixInt+1, revbinRvrs, k) = round(LEDon*7);
    Pats(12:13, negSevtFvDeg_pix+pixInt-1:negSevtFvDeg_pix+pixInt+1, revbin, k) = round(LEDon*7);
end
% repeat the entire pattern to fill the pattern file along the time axis
repNum = floor(pattern.x_num / (2*binNum));
for rep=1:repNum-1
    frIdx = rep * 2 * binNum + 1;
    toIdx = frIdx + 2 * binNum -1;
    Pats(:,:,frIdx:toIdx,k)=Pats(:,:,1:2*binNum,k);
end

%% 12 Oscillating 2x2 pixel sun. sinusoidally at .25Hz.
% 2x2 pixel sun
k = 12;
% negSevtFvDeg_pix = ceil((PIXFOR360/12)*3.5); % 26
% posSevtFvDeg_pix = ceil((PIXFOR360/12)*8.5);
binNum = 80; % half the spatial wavelength in pixels. At 80 Hz pattern refresh rate a binNum of 80 will oscillate the stripe at .5Hz.
Pats(12:13,posSevtFvDeg_pix:posSevtFvDeg_pix+1 ,1,k) = ones(2,2)*7;
Pats(12:13,negSevtFvDeg_pix-1:negSevtFvDeg_pix ,binNum+1,k) = ones(2,2)*7;

binDist=zeros(binNum+1,3);
for bin = 1:binNum
    binDist(bin+1,1) = binDist(bin,1) + sin((bin)/(binNum+1)*pi)*1/(binNum+1); % binNum+1. Integral of a sine wave
end
binDistMax = binDist((binNum+1),1);
binDist(:,1) = binDist(:,1)/binDistMax * (posSevtFvDeg_pix-negSevtFvDeg_pix+1); % pixel location relative to lowest, offset pixel

for bin = 1:binNum
    binDist(bin,2:3) = [floor(binDist(bin,1)) binDist(bin,1)- floor(binDist(bin,1))]; % [integer number-integer] (integers and fractions)
end
% plot(binDist(:,1))
% assume 2 pixels are at 1 and 1 (intensity) when binDist is at integer.
for bin = 2:binNum
    revbin = binNum + 2 - bin; % from binNum (40) to 2, [dt to 1-dt] % hardcode 1 at frame 1
    revbinRvrs = (binNum + 2 - revbin) + binNum; % from binNum + 1 (42) to 80 [1+dt to 2-dt] % hardcode 1 at frame 80
    pixInt = binDist(bin,2);
    pixFrac = binDist(bin,3);
    LEDon = ones(2,3).*[1-pixFrac 1 pixFrac];
    Pats(12:13, negSevtFvDeg_pix+pixInt-1:negSevtFvDeg_pix+pixInt+1, revbinRvrs, k) = round(LEDon*7);
    Pats(12:13, negSevtFvDeg_pix+pixInt-1:negSevtFvDeg_pix+pixInt+1, revbin, k) = round(LEDon*7);
end
% repeat the entire pattern to fill the pattern file along the time axis
repNum = floor(pattern.x_num / (2*binNum));
for rep=1:repNum-1
    frIdx = rep * 2 * binNum + 1;
    toIdx = frIdx + 2 * binNum -1;
    Pats(:,:,frIdx:toIdx,k)=Pats(:,:,1:2*binNum,k);
end
%%
ColsToCutPerSide=(PIXFOR360-NUMPIXAZIM)/2; % 32
Crop=ColsToCutPerSide+1:PIXFOR360-ColsToCutPerSide;
allPats = Pats(:,Crop,:,:);
%% 13 Contrast-Inverted stripe (DoL)
allPats(:,:,:,13) = round(.5*(LVL-allPats(:,:,:,1)));
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
ARENAAZIMUTHCENTER_PIX = (NUMPIXAZIM+1)/2;
% constants:
DEGPERRAD = 180/pi;
%% Calculate arena coordinates
rotrate_degps = ROTMAG_DEGPERFRAME * FRAMERATE_FRAMESPERSEC; % 2.25 * 60 = 120 deg/s
arenaCircumference_pix = 360.0/ARENA_DEGPERPIX;
arenaRadius_pix = arenaCircumference_pix/(2*pi);
arenaAzimuthCoordinates_deg = ([1:NUMPIXAZIM] - ARENAAZIMUTHCENTER_PIX)*ARENA_DEGPERPIX;
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
% numFrames
frames = zeros(NUMVERTPIX, NUMPIXAZIM, pattern.x_num);
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
            frames(inclinationCoordinate,azimuthCoordinate,frameInd) = 1*LVL;
        end
    end
% 	imwrite(frames(:,:,frameInd),sprintf('Roll %d .png', frameInd))
%      imshow(frames(:,:,frameInd))
%      drawnow
end
allPats(:,:,:,4) = frames;
%% 5: ROTAXIS=ROTAXIS_PITCHp;
rotMagFact = 1.5;
ROTAXIS=ROTAXIS_PITCHp;
frames = zeros(NUMVERTPIX, NUMPIXAZIM, pattern.x_num);

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
            frames(inclinationCoordinate,azimuthCoordinate,frameInd) = 1*LVL;
        end
    end
end
allPats(:,:,:,5) = frames;
%% 6: ROTAXIS=ROTAXIS_YAWp;
rotMagFact = 1;
ROTAXIS=-ROTAXIS_YAWp; % to rotate correct way (positive is yaw motion to the right)
frames = zeros(NUMVERTPIX, NUMPIXAZIM, pattern.x_num);
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
            frames(inclinationCoordinate,azimuthCoordinate,frameInd) = 1*LVL;
        end
    end
end
allPats(:,:,:,6) = frames;
%% Translation. 7
%% 7: 0 deg. From N (Progressive)
VELOCITY_METERSPERSEC = [XVELOCITY_METERSPERSEC 0 0];
frames = zeros(NUMVERTPIX, NUMPIXAZIM, pattern.x_num);
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
            frames(inclinationCoordinate,azimuthCoordinate,frameInd) = 1*LVL;
        end
    end
end
allPats(:,:,:,7) = frames;
%% invert 
% for subpat = 1:7
%     allPats(:,:,:,1) = round(LVL-allPats(:,:,:,subpat));
% end
%% 2: Starfield yaw. .5Hz sinusoidal oscillation;
k = 2;
negSixtnDeg_pix = ceil((PIXFOR360/12)*5.5); % goes through 90 deg amplitude
posSixtnDeg_pix = ceil((PIXFOR360/12)*6.5);
binNum = 40; % half the spatial wavelength in pixels. At 40 Hz pattern refresh rate a binNum of 40 will oscillate the stripe at .5Hz.
binDist=zeros(binNum+1,3);
for bin = 1:binNum
    binDist(bin+1,1) = binDist(bin,1) + sin((bin)/(binNum+1)*pi)*1/(binNum+1); % binNum+1. Integral of a sine wave
end
binDistMax = binDist((binNum+1),1);
binDist(:,1) = binDist(:,1)/binDistMax * (posSixtnDeg_pix-negSixtnDeg_pix+1); % pixel location relative to lowest, offset pixel
angPix = zeros(2*binNum,1);
angPix(1,1) = (posSixtnDeg_pix-negSixtnDeg_pix+1)*ARENA_DEGPERPIX;
for bin = 2:binNum
    revbin = binNum + 2 - bin; % from binNum (40) to 2, [dt to 1-dt] % hardcode 1 at frame 1
    revbinRvrs = (binNum + 2 - revbin) + binNum; % from binNum + 1 (42) to 80 [1+dt to 2-dt] % hardcode 1 at frame 80
    angPix(revbin,1) = binDist(bin,1)*ARENA_DEGPERPIX;
    angPix(revbinRvrs,1) = binDist(bin,1)*ARENA_DEGPERPIX;
end
plot(angPix)
rotMagFact = 1;
ROTAXIS=ROTAXIS_YAWp;
frames = zeros(NUMVERTPIX, NUMPIXAZIM, pattern.x_num);
for frameInd = 1:2*binNum
    rotmag=angPix(frameInd);
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
            frames(inclinationCoordinate,azimuthCoordinate,frameInd) = 1*LVL;
        end
    end
end
% repeat the entire pattern to fill the pattern file along the time axis
repNum = floor(pattern.x_num / (2*binNum));
for rep=1:repNum-1
    frIdx = rep * 2 * binNum + 1;
    toIdx = frIdx + 2 * binNum -1;
    frames(:,:,frIdx:toIdx)=frames(:,:,1:2*binNum);
end
allPats(:,:,:,k) = frames;
%% Prepping for PControl.m, by putting remainder in language of patterns
disp('Prepping for PControl')
pattern.Pats = allPats;
pattern.Panel_map = [12 8 4 11 7 3 10 6 2  9 5 1; 24 20 16 23 19 15 22 18 14 21 17 13; 36 32 28 35 31 27 34 30 26 33 29 25; 48 44 40 47 43 39 46 42 38 45 41 37];
pattern.BitMapIndex = process_panel_map(pattern);
% Commenting out the following line will allow you to view the pattern in the pattern player before taking the time to make them for writing to the SD card 
pattern.data = Make_pattern_vector(pattern);
% size (allPats)
thisFullFileName =  mfilename('fullpath');
[directory_name,thisFileName,thisFileExtension] = fileparts(thisFullFileName);
str = [directory_name '\Pattern_' thisFileName(6:end)];
save(str, 'pattern');
disp(['Saved to: ' str])