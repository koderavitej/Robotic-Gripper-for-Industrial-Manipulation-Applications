%% REAL STL ROBOTIC PICKER - NO BLUE OVERLAY, LEG BEND GRIP/RELEASE
% -------------------------------------------------------------------------
% This version removes the artificial blue/purple animated fingers.
%
% It uses ONLY the real STL robotic picker body on the screen.
% The lower STL finger/leg area visually bends inward to grip the object,
% then opens again to release it.
%
% Sequence:
%   1) real STL fingers/legs are open
%   2) robot moves toward object
%   3) lower STL fingers/legs bend inward and grip object
%   4) object moves with robot after safe grip
%   5) robot stops / rotates object
%   6) STL fingers/legs open again and release object
%
% Important:
% Your current STL is one complete mesh. Exact physical hinge rotation needs
% separate STL files for each finger. This code avoids the fake blue overlay
% and uses visual deformation of the real STL lower finger/leg region.
%
% Base MATLAB only.
% -------------------------------------------------------------------------

clear; clc; close all;

%% ============================ SETTINGS ===================================
objectTypes = {'cube','square','circular','rectangular'};  % cube, square, circular cylinder, rectangular block
rotationTargetDeg = 180;               % 90, 180, or 360
saveVideo = true;                     % set true if you want MP4 video
animationSpeed = 0.015;
videoFrameRate = 24;
maxFacesForFastPlot = 50000;

% If object is still slightly off, adjust only these two values:
objectXYOffset = [1.0 -4.0];              % fixed same center for cube and square
objectZFactor  = 0.18;                  % fixed same vertical position for all objects
objectSizeFactor = 0.230;               % fixed length/width footprint for every object

objectMass_kg = 0.35;
frictionCoefficient = 0.65;
safetyFactor = 1.5;

%% ============================ FOLDERS ====================================
scriptPath = mfilename('fullpath');
if isempty(scriptPath)
    baseDir = pwd;
else
    baseDir = fileparts(scriptPath);
end

outDir  = fullfile(baseDir, 'outputs_FINAL_FOUR_OBJECTS_CYLINDER_WIDTH_FIXED_SIZE');
snapDir = fullfile(outDir, 'snapshots');
figDir  = fullfile(outDir, 'graphs');
safeMkdir(outDir); safeMkdir(snapDir); safeMkdir(figDir);

%% ============================ LOAD STL ===================================
stlFile = findSTLFile(baseDir);
fprintf('\nLoading real STL robotic picker:\n%s\n', stlFile);

[F, V] = readSTLCompat(stlFile);
F = double(F);
V = double(V);

fprintf('Original STL: %d faces, %d vertices\n', size(F,1), size(V,1));

if size(F,1) > maxFacesForFastPlot
    fprintf('Reducing mesh for smoother animation...\n');
    try
        [F, V] = reducepatch(F, V, maxFacesForFastPlot/size(F,1));
    catch
        warning('Mesh reduction failed. Using original STL mesh.');
    end
end
fprintf('Using mesh: %d faces, %d vertices\n', size(F,1), size(V,1));

bboxMin = min(V,[],1);
bboxMax = max(V,[],1);
bboxSize = bboxMax - bboxMin;
modelCenter = (bboxMin + bboxMax)/2;
V0 = V - modelCenter;

robotHeight = max(bboxSize);
robotWidth  = max(bboxSize(1:2));

objectSize = max(robotWidth*objectSizeFactor, 20);
objectCenterLocal = [objectXYOffset(1), objectXYOffset(2), min(V0(:,3)) + objectZFactor*robotHeight];

%% ============================ GRIP MODEL =================================
degSymbol = char(176);

P.contactStartRatio = 0.40;
P.pinIndex = 3;
P.backSpringK_N_per_mm = 0.45;
P.innerSpringK_N_per_mm = 0.35;
P.maxBackSpringCompression_mm = 10;
P.maxInnerSpringCompression_mm = 7;
P.pinPreload_N = [0.4 1.2 2.0];

requiredForce = objectMass_kg*9.81*safetyFactor/max(frictionCoefficient,eps);
finalGripForce = 3*(P.backSpringK_N_per_mm*P.maxBackSpringCompression_mm + ...
                    P.innerSpringK_N_per_mm*P.maxInnerSpringCompression_mm + ...
                    P.pinPreload_N(P.pinIndex));

% Robot movement
approachShift = [0 0 38];
gripShift = [0 0 0];
targetShift = [95 25 45];
releaseLift = [0 0 35];

fprintf('\n============================================================\n');
fprintf('SMALL OBJECT + WIDER FINGERS 3D VIDEO STARTED\n');
fprintf('Required holding force: %.2f N\n', requiredForce);
fprintf('Final estimated grip force: %.2f N\n', finalGripForce);
fprintf('Output folder:\n%s\n', outDir);
fprintf('============================================================\n\n');

%% ============================ FIGURE =====================================
fig = figure('Name','Real STL Robotic Picker - No Blue Overlay', ...
    'Color','w','Position',[35 45 1450 830]);

ax = axes('Parent',fig,'Position',[0.04 0.08 0.64 0.84]);
hold(ax,'on'); grid(ax,'on'); axis(ax,'equal'); axis(ax,'vis3d');
xlabel(ax,'X (mm)'); ylabel(ax,'Y (mm)'); zlabel(ax,'Z (mm)');
title(ax,'Real STL robotic picker: lower fingers bend, grip and release object', ...
    'FontSize',13,'FontWeight','bold');

xlim(ax,[-0.85*robotHeight, 1.45*robotHeight]);
ylim(ax,[-0.95*robotHeight, 0.95*robotHeight]);
zlim(ax,[-0.92*robotHeight, 1.00*robotHeight]);
view(ax,135,24);
camlight(ax,'headlight');
lighting(ax,'gouraud');
material(ax,'dull');
cleanAxes(ax);

robotPatch = patch(ax,'Faces',F,'Vertices',V0, ...
    'FaceColor',[0.05 0.58 0.65], ...
    'EdgeColor','none', ...
    'FaceLighting','gouraud', ...
    'AmbientStrength',0.35, ...
    'DiffuseStrength',0.82);

[objF,objV0] = makeObjectMesh('cube', objectSize);
objPatch = patch(ax,'Faces',objF,'Vertices',objV0 + objectCenterLocal, ...
    'FaceColor',[0.98 0.92 0.55], ...
    'EdgeColor','k', ...
    'LineWidth',1.1, ...
    'FaceAlpha',0.94);

% A simple black motion path only, no red/blue overlay
pathPts = [linspace(objectCenterLocal(1),objectCenterLocal(1)+targetShift(1),80)', ...
           linspace(objectCenterLocal(2),objectCenterLocal(2)+targetShift(2),80)', ...
           linspace(objectCenterLocal(3),objectCenterLocal(3)+targetShift(3),80)'];
plot3(ax,pathPts(:,1),pathPts(:,2),pathPts(:,3),'k:','LineWidth',1.5);
text(ax,objectCenterLocal(1)+45,objectCenterLocal(2)+48,objectCenterLocal(3)+42, ...
    '3D path after safe grip','FontSize',9);

panel = axes('Parent',fig,'Position',[0.71 0.07 0.26 0.87]);
axis(panel,[0 1 0 1]); axis(panel,'off'); cleanAxes(panel);

%% ============================ VIDEO ======================================
videoObj = [];
if saveVideo
    try
        videoPath = fullfile(outDir,'real_STL_SMALL_OBJECT_WIDE_FINGERS.mp4');
        videoObj = VideoWriter(videoPath,'MPEG-4');
        videoObj.FrameRate = videoFrameRate;
        open(videoObj);
    catch
        warning('VideoWriter failed; animation will still run.');
        videoObj = [];
    end
end

history = struct('frame',[],'object',{{}},'phase',{{}},'closeRatio',[], ...
    'motorAngle',[],'backSpring',[],'innerSpring',[],'gripForce',[], ...
    'gripMargin',[],'objectRotation',[]);
frameNo = 0;

%% ============================ MAIN LOOP ==================================
for objIndex = 1:numel(objectTypes)
    objType = objectTypes{objIndex};
    [objF,objV0] = makeObjectMesh(objType, objectSize);
    set(objPatch,'Faces',objF,'Vertices',objV0 + objectCenterLocal);

    fprintf('Running %s...\n', upper(objType));

    % 1. Open
    for k = 1:20
        [frameNo,history] = updateFrame(frameNo,history,objType,fig,ax,panel,robotPatch,objPatch, ...
            V0,objV0,objectCenterLocal,P,requiredForce,degSymbol, ...
            'Fingers open - ready to approach',0,0,0,approachShift,0,objectCenterLocal, ...
            objectSize,animationSpeed,videoObj);
    end
    safeSave(fig,fullfile(snapDir,['snapshot_' objType '_01_open.png']));

    % 2. Approach
    nApproach = 40;
    for k = 1:nApproach
        t = (k-1)/(nApproach-1);
        shift = (1-t)*approachShift + t*gripShift;
        [frameNo,history] = updateFrame(frameNo,history,objType,fig,ax,panel,robotPatch,objPatch, ...
            V0,objV0,objectCenterLocal,P,requiredForce,degSymbol, ...
            'Open robot moving toward object',0,0,0,shift,0,objectCenterLocal, ...
            objectSize,animationSpeed,videoObj);
    end
    safeSave(fig,fullfile(snapDir,['snapshot_' objType '_02_approach.png']));

    % 3. Close / grip
    nClose = 75;
    for k = 1:nClose
        c = (k-1)/(nClose-1);
        motorAngle = 78*c;
        [backSpring,innerSpring,gripForce,gripMargin] = gripValues(c,P,requiredForce);

        if c < P.contactStartRatio
            phaseText = 'Lower STL fingers closing toward object';
        elseif gripMargin < 1
            phaseText = 'Fingers contacting object - springs tightening';
        else
            phaseText = 'Object safely gripped';
        end

        [frameNo,history] = updateFrame(frameNo,history,objType,fig,ax,panel,robotPatch,objPatch, ...
            V0,objV0,objectCenterLocal,P,requiredForce,degSymbol, ...
            phaseText,c,motorAngle,0,gripShift,0,objectCenterLocal, ...
            objectSize,animationSpeed,videoObj);
    end
    safeSave(fig,fullfile(snapDir,['snapshot_' objType '_03_grip.png']));

    % 4. Hold
    for k = 1:20
        [frameNo,history] = updateFrame(frameNo,history,objType,fig,ax,panel,robotPatch,objPatch, ...
            V0,objV0,objectCenterLocal,P,requiredForce,degSymbol, ...
            'Holding object firmly',1,78,0,gripShift,0,objectCenterLocal, ...
            objectSize,animationSpeed,videoObj);
    end

    % 5. Move while gripped - object follows exact same robot shift
    nMove = 60;
    for k = 1:nMove
        t = (k-1)/(nMove-1);
        shift = targetShift*t;
        objC = objectCenterLocal + shift;
        [frameNo,history] = updateFrame(frameNo,history,objType,fig,ax,panel,robotPatch,objPatch, ...
            V0,objV0,objectCenterLocal,P,requiredForce,degSymbol, ...
            'Moving object while gripped',1,78,0,shift,0,objC, ...
            objectSize,animationSpeed,videoObj);
    end
    safeSave(fig,fullfile(snapDir,['snapshot_' objType '_04_move.png']));

    % 6. Rotate while gripped
    % IMPORTANT FIX:
    % During rotation, the object center must rotate with the gripper.
    % Otherwise the robot rotates but the object stays at old center, causing a gap.
    nRot = 60;
    for k = 1:nRot
        t = (k-1)/(nRot-1);
        rot = rotationTargetDeg*t;
        Rhold = rotzLocal(rot);
        objC = (Rhold*objectCenterLocal')' + targetShift;  % object follows rotating gripper
        [frameNo,history] = updateFrame(frameNo,history,objType,fig,ax,panel,robotPatch,objPatch, ...
            V0,objV0,objectCenterLocal,P,requiredForce,degSymbol, ...
            sprintf('Rotating gripper and held object together: %.0f%s',rot,degSymbol),1,78,rot,targetShift,rot,objC, ...
            objectSize,animationSpeed,videoObj);
    end
    safeSave(fig,fullfile(snapDir,['snapshot_' objType '_05_rotate_gap_fixed.png']));

    % 7. Release sequence - no gap while object is still marked as held.
    % Object is left at the SAME rotated gripper position.
    % This prevents the object from jumping or showing a gap when release starts.
    nRelease = 70;
    holdPart = 0.35;   % first 35% of release stage: still tight/closed
    Rrelease = rotzLocal(rotationTargetDeg);
    releaseObjCenter = (Rrelease*objectCenterLocal')' + targetShift;
    for k = 1:nRelease
        t = (k-1)/(nRelease-1);
        objC = releaseObjCenter;

        if t < holdPart
            % Still holding: keep visual fingers fully closed so no gap appears.
            c = 1.0;
            motorAngle = 78;
            phaseText = 'Robot stopped - object still tightly held';
        else
            % Now release: open the real STL fingers and leave object at target.
            releaseT = (t - holdPart) / (1 - holdPart);
            c = max(0, 1 - releaseT);
            motorAngle = 78*c;
            if c > P.contactStartRatio
                phaseText = 'Starting release - object still supported';
            elseif c > 0.05
                phaseText = 'Object released - fingers opening away';
            else
                phaseText = 'Object released at target';
            end
        end

        [frameNo,history] = updateFrame(frameNo,history,objType,fig,ax,panel,robotPatch,objPatch, ...
            V0,objV0,objectCenterLocal,P,requiredForce,degSymbol, ...
            phaseText,c,motorAngle,rotationTargetDeg,targetShift,rotationTargetDeg,objC, ...
            objectSize,animationSpeed,videoObj);
    end
    safeSave(fig,fullfile(snapDir,['snapshot_' objType '_06_release_gap_fixed.png']));

    % 8. Move robot away after release, object stays at the rotated target position
    nAway = 35;
    for k = 1:nAway
        t = (k-1)/(nAway-1);
        shift = targetShift + releaseLift*t;
        objC = releaseObjCenter;
        [frameNo,history] = updateFrame(frameNo,history,objType,fig,ax,panel,robotPatch,objPatch, ...
            V0,objV0,objectCenterLocal,P,requiredForce,degSymbol, ...
            'Open robot moved away after release',0,0,rotationTargetDeg,shift,rotationTargetDeg,objC, ...
            objectSize,animationSpeed,videoObj);
    end
    safeSave(fig,fullfile(snapDir,['snapshot_' objType '_07_done.png']));
end

if ~isempty(videoObj)
    try
        close(videoObj);
        fprintf('Video saved: %s\n',videoPath);
    catch
    end
end

%% ============================ SAVE RESULTS ===============================
T = table(history.frame,history.object,history.phase,history.closeRatio,history.motorAngle, ...
    history.backSpring,history.innerSpring,history.gripForce,history.gripMargin,history.objectRotation, ...
    'VariableNames',{'Frame','ObjectType','Phase','FingerClosingRatio','MotorAngle_deg', ...
    'BackSpringCompression_mm','InnerBarSpringCompression_mm','GripForce_N','GripSafetyMargin','ObjectRotation_deg'});
writetable(T,fullfile(outDir,'no_blue_leg_bend_history.csv'));
createGraphs(figDir,history,degSymbol);

fprintf('\nDone. Open this folder:\n%s\n',outDir);

%% ========================================================================
% FUNCTIONS
% ========================================================================

function [frameNo,history] = updateFrame(frameNo,history,objType,fig,ax,panel,robotPatch,objPatch, ...
    V0,objV0,objectCenterLocal,P,requiredForce,degSymbol,phaseText,closeRatio,motorAngle, ...
    objectRot,robotShift,robotRot,objectCenterWorld,objectSize,pauseTime,videoObj)

    frameNo = frameNo + 1;
    [backSpring,innerSpring,gripForce,gripMargin] = gripValues(closeRatio,P,requiredForce);
    contactRatio = max((closeRatio-P.contactStartRatio)/(1-P.contactStartRatio),0);

    % Real STL lower finger/leg visual bending
    Vdef = deformRealSTLFingers(V0,closeRatio,objectCenterLocal,objectSize,objType);
    Rrobot = rotzLocal(robotRot);
    Vworld = (Rrobot*Vdef')' + robotShift;

    % Final visual safety: keep real STL finger/leg vertices OUTSIDE the object.
    % This fixes the cube-inside issue while keeping square contact close.
    RobjForContact = rotzLocal(objectRot);
    Vworld = keepRobotOutsideObject(Vworld, objectCenterWorld, RobjForContact, objType, objectSize, closeRatio);

    set(robotPatch,'Vertices',Vworld);

    % Object follows robot only after gripped and stays at release target
    squeeze = 1 - 0.06*contactRatio;
    if strcmpi(objType,'square')
        S = diag([squeeze squeeze 1.0]);
    else
        S = diag([squeeze squeeze squeeze]);
    end
    Robj = rotzLocal(objectRot);
    objV = (Robj*(S*objV0') )' + objectCenterWorld;
    set(objPatch,'Vertices',objV);

    if gripMargin >= 1 && closeRatio > 0.85
        set(objPatch,'FaceColor',[0.66 1.00 0.68]);
    elseif contactRatio > 0
        set(objPatch,'FaceColor',[1.00 0.82 0.35]);
    else
        set(objPatch,'FaceColor',[0.98 0.92 0.55]);
    end

    drawPanel(panel,objType,phaseText,closeRatio,motorAngle,backSpring,innerSpring,gripForce, ...
        gripMargin,objectRot,degSymbol,requiredForce);

    title(ax,sprintf('Real STL robotic picker | %s | lower STL finger bend %.0f%%',phaseText,closeRatio*100), ...
        'FontSize',12,'FontWeight','bold');
    drawnow;
    pause(pauseTime);

    if ~isempty(videoObj)
        try
            writeVideo(videoObj,getframe(fig));
        catch
        end
    end

    history.frame(end+1,1) = frameNo;
    history.object{end+1,1} = objType;
    history.phase{end+1,1} = phaseText;
    history.closeRatio(end+1,1) = closeRatio;
    history.motorAngle(end+1,1) = motorAngle;
    history.backSpring(end+1,1) = backSpring;
    history.innerSpring(end+1,1) = innerSpring;
    history.gripForce(end+1,1) = gripForce;
    history.gripMargin(end+1,1) = gripMargin;
    history.objectRotation(end+1,1) = objectRot;
end

function Vd = deformRealSTLFingers(V0,c,objC,objSize,objType)
    % Same closing motion for cube, square, circular and rectangular with the same length/width objects.
    % This avoids a square gap and prevents cube penetration.

    Vd = V0;

    zMin = min(V0(:,3));
    zMax = max(V0(:,3));

    % Move only lower finger/leg area. Top frame remains unchanged.
    zCut = zMin + 0.38*(zMax-zMin);
    lower = V0(:,3) < zCut;

    w = zeros(size(V0,1),1);
    w(lower) = ((zCut - V0(lower,3))/(zCut-zMin+eps)).^1.20;
    w = min(max(w,0),1);

    xy = V0(:,1:2) - objC(1:2);
    d = sqrt(sum(xy.^2,2)) + eps;
    inward = -xy ./ d;

    % Same opening behavior for all objects, with extra closing only for cylinder
    % so the round surface does not show a visible gap.
    openSpread = objSize * 0.17 * (1-c);
    closePull  = objSize * 0.30 * c;

    if strcmpi(objType,'circular') || strcmpi(objType,'circle') || strcmpi(objType,'cylinder')
        closePull = objSize * 0.39 * c;      % extra tightening for curved object
        openSpread = objSize * 0.15 * (1-c);
    end

    moveAmount = (closePull - openSpread) .* w;
    Vd(:,1:2) = V0(:,1:2) + moveAmount .* inward;

    % Very small vertical correction only.
    Vd(:,3) = V0(:,3) + objSize*0.006*c.*w;
end


function Vout = keepRobotOutsideObject(Vin,C,Robj,objType,objSize,closeRatio)
    % Prevents the visual STL fingers from appearing inside the object.
    % Works for cube, square, rectangular block, and circular/cylinder object.

    Vout = Vin;

    if closeRatio <= 0.02
        return;
    end

    objType = lower(strtrim(objType));
    local = (Robj'*(Vin - C)')';
    clearance = 0.025*objSize;

    if strcmp(objType,'circular') || strcmp(objType,'circle') || strcmp(objType,'cylinder')
        clearance = 0.005*objSize;  % smaller clearance for cylinder contact
        dims = objectDimensions(objType,objSize);
        radius = dims(1)/2 + clearance;
        hz = dims(3)/2 + clearance;

        r = sqrt(local(:,1).^2 + local(:,2).^2) + eps;
        inside = (r < radius) & (abs(local(:,3)) < hz);

        if any(inside)
            local(inside,1) = local(inside,1)./r(inside)*radius;
            local(inside,2) = local(inside,2)./r(inside)*radius;
        end
    else
        dims = objectDimensions(objType,objSize);

        hx = dims(1)/2 + clearance;
        hy = dims(2)/2 + clearance;
        hz = dims(3)/2 + clearance;

        inside = abs(local(:,1)) < hx & abs(local(:,2)) < hy & abs(local(:,3)) < hz;

        if any(inside)
            idx = find(inside);
            for n = 1:numel(idx)
                i = idx(n);

                % Push out in XY only because the fingers are side contacts.
                dx = hx - abs(local(i,1));
                dy = hy - abs(local(i,2));

                if dx < dy
                    if local(i,1) >= 0
                        local(i,1) = hx;
                    else
                        local(i,1) = -hx;
                    end
                else
                    if local(i,2) >= 0
                        local(i,2) = hy;
                    else
                        local(i,2) = -hy;
                    end
                end
            end
        end
    end

    if exist('inside','var') && any(inside)
        Vout(inside,:) = (Robj*local(inside,:)')' + C;
    end
end


function [backSpring,innerSpring,gripForce,gripMargin] = gripValues(c,P,requiredForce)
    contactRatio = max((c-P.contactStartRatio)/(1-P.contactStartRatio),0);
    innerRatio = max((c-0.62)/(1-0.62),0);
    backSpring = P.maxBackSpringCompression_mm*contactRatio;
    innerSpring = P.maxInnerSpringCompression_mm*innerRatio;
    gripForce = 3*(P.backSpringK_N_per_mm*backSpring + ...
                   P.innerSpringK_N_per_mm*innerSpring + ...
                   P.pinPreload_N(P.pinIndex)*c);
    gripMargin = gripForce/max(requiredForce,eps);
end

function drawPanel(ax,objType,phaseText,c,motorAngle,backSpring,innerSpring,gripForce,gripMargin,objectRot,degSymbol,requiredForce)
    cla(ax); hold(ax,'on'); axis(ax,[0 1 0 1]); axis(ax,'off');

    if gripMargin >= 1 && c > 0.85
        state = 'SAFE GRIP / LOCKED';
        stateColor = [0.10 0.55 0.20];
    elseif c < 0.05
        state = 'OPEN / RELEASED';
        stateColor = [0.08 0.32 0.62];
    else
        state = 'REAL STL FINGERS CLOSING / OPENING';
        stateColor = [0.86 0.46 0.05];
    end

    rectangle(ax,'Position',[0 0 1 1],'Curvature',0.02,'FaceColor',[0.97 0.98 1],'EdgeColor',[0.65 0.65 0.65]);
    rectangle(ax,'Position',[0 0.91 1 0.09],'FaceColor',[0.02 0.26 0.43],'EdgeColor',[0.02 0.26 0.43]);
    text(ax,0.04,0.955,'AI + KINEMATIC CONTROL PANEL','Color','w','FontSize',12,'FontWeight','bold','VerticalAlignment','middle');

    rectangle(ax,'Position',[0.04 0.80 0.92 0.09],'Curvature',0.03,'FaceColor','w','EdgeColor',[0.75 0.75 0.75]);
    text(ax,0.06,0.848,'Current phase','FontSize',9,'Color',[0.35 0.35 0.35],'FontWeight','bold');
    text(ax,0.06,0.813,phaseText,'FontSize',8.4,'FontWeight','bold');

    rectangle(ax,'Position',[0.04 0.64 0.92 0.12],'Curvature',0.03,'FaceColor',[0.93 0.97 1],'EdgeColor',[0.55 0.70 0.85]);
    text(ax,0.06,0.713,['Object: ',upper(objType)],'FontSize',10,'FontWeight','bold');
    text(ax,0.06,0.678,'Smaller object fits between wider real STL fingers','FontSize',8.3);
    text(ax,0.06,0.645,'Object follows gripper after safe grip','FontSize',8.3);

    rectangle(ax,'Position',[0.04 0.36 0.92 0.24],'Curvature',0.03,'FaceColor','w','EdgeColor',[0.75 0.75 0.75]);
    y0 = 0.565; dy = 0.030;
    metricRow(ax,y0,      'Motor input angle', sprintf('%.1f%s',motorAngle,degSymbol));
    metricRow(ax,y0-dy,   'Real STL finger bend', sprintf('%.0f%%',c*100));
    metricRow(ax,y0-2*dy, 'Back spring', sprintf('%.1f mm',backSpring));
    metricRow(ax,y0-3*dy, 'Inner bar spring', sprintf('%.1f mm',innerSpring));
    metricRow(ax,y0-4*dy, 'Total grip force', sprintf('%.2f N',gripForce));
    metricRow(ax,y0-5*dy, 'Required force', sprintf('%.2f N',requiredForce));
    metricRow(ax,y0-6*dy, 'Safety margin', sprintf('%.2f x',gripMargin));
    metricRow(ax,y0-7*dy, 'Object rotation', sprintf('%.0f%s',objectRot,degSymbol));

    rectangle(ax,'Position',[0.04 0.29 0.92 0.055],'Curvature',0.03,'FaceColor',stateColor,'EdgeColor',stateColor);
    text(ax,0.06,0.318,['State: ',state],'FontSize',9.4,'FontWeight','bold','Color','w','VerticalAlignment','middle');

    rectangle(ax,'Position',[0.04 0.17 0.92 0.09],'Curvature',0.03,'FaceColor','w','EdgeColor',[0.75 0.75 0.75]);
    text(ax,0.06,0.225,'Sequence: wide open -> close around object -> tighten -> move -> release','FontSize',7.7,'FontWeight','bold');
    text(ax,0.06,0.195,'Cube, square, circular, and rectangular objects tested','FontSize',7.8);

    progressBar(ax,0.06,0.102,0.86,0.022,c,'STL finger bend');
    progressBar(ax,0.06,0.062,0.86,0.022,min(backSpring/10,1),'Back spring');
    progressBar(ax,0.06,0.022,0.86,0.022,min(innerSpring/7,1),'Inner spring');
end

function metricRow(ax,y,label,value)
    text(ax,0.07,y,label,'FontSize',8.2,'Color',[0.25 0.25 0.25],'VerticalAlignment','middle');
    text(ax,0.92,y,value,'FontSize',8.5,'Color',[0.03 0.03 0.03],'FontWeight','bold','HorizontalAlignment','right','VerticalAlignment','middle');
end

function progressBar(ax,x,y,w,h,val,label)
    val = min(max(val,0),1);
    text(ax,x,y+h+0.004,sprintf('%s: %.0f%%',label,val*100),'FontSize',7.8);
    rectangle(ax,'Position',[x y w h],'Curvature',0.02,'FaceColor',[0.90 0.92 0.94],'EdgeColor',[0.70 0.70 0.70]);
    rectangle(ax,'Position',[x y w*val h],'Curvature',0.02,'FaceColor',[0.10 0.45 0.80],'EdgeColor','none');
end

function [F,V] = makeObjectMesh(type,s)
    type = lower(strtrim(type));
    dims = objectDimensions(type,s);

    if strcmp(type,'circular') || strcmp(type,'circle') || strcmp(type,'cylinder')
        % Circular object: diameter uses the SAME fixed length/width as all objects.
        radius = dims(1)/2;
        height = dims(3);
        n = 48;
        th = linspace(0,2*pi,n+1);
        th(end) = [];

        bottom = [radius*cos(th(:)), radius*sin(th(:)), -height/2*ones(n,1)];
        top    = [radius*cos(th(:)), radius*sin(th(:)),  height/2*ones(n,1)];

        V = [bottom; top; 0 0 -height/2; 0 0 height/2];
        bottomCenter = 2*n + 1;
        topCenter = 2*n + 2;

        F = [];
        for i = 1:n
            j = mod(i,n) + 1;
            F = [F; i j n+j; i n+j n+i]; %#ok<AGROW>
            F = [F; bottomCenter j i; topCenter n+i n+j]; %#ok<AGROW>
        end
    else
        a = dims/2;
        V = [-a(1) -a(2) -a(3); a(1) -a(2) -a(3); a(1) a(2) -a(3); -a(1) a(2) -a(3); ...
             -a(1) -a(2)  a(3); a(1) -a(2)  a(3); a(1) a(2)  a(3); -a(1) a(2)  a(3)];
        F = [1 2 3;1 3 4;5 8 7;5 7 6;1 5 6;1 6 2; ...
             2 6 7;2 7 3;3 7 8;3 8 4;4 8 5;4 5 1];
    end
end

function dims = objectDimensions(type,s)
    % IMPORTANT:
    % All objects use the SAME length and SAME width footprint.
    % This prevents gap/inside issues because the gripper sees the same contact size.
    fixedLength = s*0.92;
    fixedWidth  = s*0.92;

    switch lower(strtrim(type))
        case 'cube'
            % Cube: length = width = height.
            dims = [fixedLength fixedWidth fixedLength];

        case 'square'
            % Square block: same length/width, smaller height.
            dims = [fixedLength fixedWidth fixedLength*0.55];

        case {'circular','circle','cylinder'}
            % Circular object / cylinder:
            % A round object needs a slightly larger fixed diameter to touch
            % the same gripper fingers because it has curved sides, not flat faces.
            cylinderDiameter = fixedLength*1.14;
            dims = [cylinderDiameter cylinderDiameter fixedLength*0.75];

        case 'rectangular'
            % Rectangular block for gripper testing:
            % Same length/width footprint, different height so it is still a rectangular prism.
            dims = [fixedLength fixedWidth fixedLength*0.70];

        otherwise
            dims = [fixedLength fixedWidth fixedLength*0.55];
    end
end


function createGraphs(figDir,history,degSymbol)
    % ---------------------------------------------------------------------
    % ASSIGNMENT RESULT FIGURES
    % This section creates the result figures for all points in the project:
    % 1) Kinematics of finger mechanism
    % 2) Input-output relationship
    % 3) Dimension optimization
    % 4) Performance comparison
    % 5) Final optimized design
    % 6) Grasping performance
    % 7) Before vs after comparison
    % ---------------------------------------------------------------------

    %% 1. Kinematics of the finger mechanism
    f = figure('Name','Result 1 - Kinematics of finger mechanism','Color','w','Position',[100 80 950 620]);
    ax = axes('Parent',f); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');

    O = [-10 -20];              % contact bar pivot
    H = [0 0];                  % finger hinge
    Bopen = [-10 5];            % contact bar open position
    Bclosed = [-2 2];           % contact bar pressed position
    Jopen = [1.5 21];           % pushrod open joint
    Jclosed = [8 18];           % pushrod closed joint
    TipOpen = [45 0];           % fingertip open
    TipClosed = [18 41];        % fingertip closed/wrapped

    plot(ax,[O(1) Bopen(1)],[O(2) Bopen(2)],'--','LineWidth',3,'DisplayName','contact bar open position');
    plot(ax,[O(1) Bclosed(1)],[O(2) Bclosed(2)],'-','LineWidth',3,'DisplayName','contact bar motion');
    plot(ax,[Bclosed(1) Jclosed(1)],[Bclosed(2) Jclosed(2)],'-','LineWidth',3,'DisplayName','linkage/pushrod motion');
    plot(ax,[H(1) TipOpen(1)],[H(2) TipOpen(2)],'--','LineWidth',4,'DisplayName','fingertip open position');
    plot(ax,[H(1) TipClosed(1)],[H(2) TipClosed(2)],'-','LineWidth',4,'DisplayName','passive fingertip rotation');

    theta = linspace(0,65.7,120);
    pathX = 45*cosd(theta);
    pathY = 45*sind(theta);
    plot(ax,pathX,pathY,':','LineWidth',2.5,'DisplayName','fingertip path during gripping');

    plot(ax,O(1),O(2),'ko','MarkerFaceColor','k');
    plot(ax,H(1),H(2),'ko','MarkerFaceColor','k');
    text(ax,O(1)-10,O(2)-5,'Contact-bar pivot O','FontSize',10);
    text(ax,H(1)+3,H(2)-8,'Hinge H','FontSize',10);
    text(ax,28,36,'Fingertip path','FontSize',11);
    title(ax,'Kinematics of the finger mechanism','FontSize',16,'FontWeight','normal');
    xlabel(ax,'x position (mm)'); ylabel(ax,'y position (mm)');
    legend(ax,'Location','northwest');
    xlim(ax,[-25 52]); ylim(ax,[-35 48]);
    cleanAxes(ax); safeSave(f,fullfile(figDir,'result_01_kinematics_finger_mechanism.png'));

    %% 2. Input-output relationship
    contactBarDisp = linspace(0,11.1,30);
    fingertipAngle = 65.7*(1-exp(-contactBarDisp/5.2));
    keyX = [0 2.2 4.0 6.2 8.6 11.1];
    keyY = interp1(contactBarDisp,fingertipAngle,keyX,'pchip');

    f = figure('Name','Result 2 - Input-output relationship','Color','w','Position',[100 80 950 560]);
    ax = axes('Parent',f); hold(ax,'on'); grid(ax,'on');
    plot(ax,contactBarDisp,fingertipAngle,'-o','LineWidth',2.4,'MarkerSize',6);
    for i = 1:numel(keyX)
        text(ax,keyX(i),keyY(i)+2.2,sprintf('%.1f%s',keyY(i),degSymbol), ...
            'HorizontalAlignment','center','FontSize',9.5);
    end
    title(ax,'Input-output relationship: contact-bar displacement vs fingertip angle','FontSize',16,'FontWeight','normal');
    xlabel(ax,'Contact-bar arc displacement (mm)');
    ylabel(ax,['Passive fingertip bending angle ', char(952), ' (degrees)']);
    xlim(ax,[-0.5 11.8]); ylim(ax,[-3 70]);
    cleanAxes(ax); safeSave(f,fullfile(figDir,'result_02_input_output_relationship.png'));

    %% 3. Dimension optimization
    designNames = {'D1','D2','D3','D4','D5'};
    Lc = [20 23 24 25 27];          % contact bar length
    Lp = [16.5 18.2 19.0 19.69 21.0]; % pushrod length
    Lt = [17 19 20 21 22];          % fingertip tab length
    finalAngle = [78.9 73.2 73.9 65.7 68.3];

    f = figure('Name','Result 3 - Dimension optimization','Color','w','Position',[100 80 950 560]);
    ax = axes('Parent',f); hold(ax,'on'); grid(ax,'on');
    yyaxis(ax,'left');
    plot(ax,1:5,Lc,'-o','LineWidth',2.2,'DisplayName','contact bar length Lc');
    plot(ax,1:5,Lp,'-s','LineWidth',2.2,'DisplayName','pushrod length Lp');
    plot(ax,1:5,Lt,'-^','LineWidth',2.2,'DisplayName','fingertip tab length Lt');
    ylabel(ax,'Dimension value (mm)');
    yyaxis(ax,'right');
    plot(ax,1:5,finalAngle,'-d','LineWidth',2.2,'DisplayName','final fingertip angle');
    ylabel(ax,'Final fingertip angle (degrees)');
    set(ax,'XTick',1:5,'XTickLabel',designNames);
    title(ax,'Dimension optimization of contact-bar linkage parameters','FontSize',16,'FontWeight','normal');
    xlabel(ax,'Tested design option');
    legend(ax,'Location','best');
    cleanAxes(ax); safeSave(f,fullfile(figDir,'result_03_dimension_optimization.png'));

    %% 4. Performance comparison
    % Score combines smoothness, enough bending, no locking, and object wrapping.
    smoothness = [42 61 64 92 89];
    bending   = [85 78 80 70 73];
    noLocking = [20 50 52 95 92];
    wrapping  = [22 40 36 96 89];
    performanceScores = 0.25*(smoothness + bending + noLocking + wrapping);

    f = figure('Name','Result 4 - Performance comparison','Color','w','Position',[100 80 950 560]);
    ax = axes('Parent',f); hold(ax,'on'); grid(ax,'on');
    bar(ax,performanceScores,0.8);
    set(ax,'XTick',1:5,'XTickLabel',designNames);
    for i = 1:numel(performanceScores)
        text(ax,i,performanceScores(i)+2,sprintf('%.1f',performanceScores(i)), ...
            'HorizontalAlignment','center','FontSize',10);
    end
    title(ax,'Performance comparison for tested linkage dimensions','FontSize',16,'FontWeight','normal');
    xlabel(ax,'Tested design option');
    ylabel(ax,'Total performance score (%)');
    ylim(ax,[0 100]);
    cleanAxes(ax); safeSave(f,fullfile(figDir,'result_04_performance_comparison.png'));

    %% 5. Final optimized design
    bestIndex = 4; % D4 selected because it has high score and avoids locking
    f = figure('Name','Result 5 - Final optimized design','Color','w','Position',[100 80 700 620]);
    ax = axes('Parent',f); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');

    O = [-10 -20]; H = [0 0]; B = [-10 5]; J = [1.5 21]; Tip = [18 41];
    plot(ax,[O(1) B(1)],[O(2) B(2)],'-o','LineWidth',3,'DisplayName','contact bar');
    plot(ax,[B(1) J(1)],[B(2) J(2)],'-o','LineWidth',3,'DisplayName','pushrod');
    plot(ax,[H(1) J(1)],[H(2) J(2)],'-o','LineWidth',3,'DisplayName','fingertip tab');
    plot(ax,[H(1) Tip(1)],[H(2) Tip(2)],'-o','LineWidth',4,'DisplayName','fingertip link');

    text(ax,-17,-5,'O = (-10, -20) mm','HorizontalAlignment','right');
    text(ax,-14,12,'Finger hinge H');
    text(ax,20,8,'Lp = 19.69 mm');
    text(ax,16,-12,'Lt = 21 mm');
    text(ax,8,-34,'Lc = 25 mm');
    title(ax,'Final optimized dimensions used in MATLAB model','FontSize',16,'FontWeight','normal');
    xlabel(ax,'x (mm)'); ylabel(ax,'y (mm)');
    xlim(ax,[-18 25]); ylim(ax,[-25 45]);
    legend(ax,'Location','best');
    cleanAxes(ax); safeSave(f,fullfile(figDir,'result_05_final_optimized_design.png'));

    %% 6. Grasping performance with sphere/cylinder/cube
    objectNames = {'Sphere','Cylinder','Cube'};
    contactAngles = [65.7 63.0 53.9];

    f = figure('Name','Result 6 - Grasping performance','Color','w','Position',[100 80 1100 460]);
    for k = 1:3
        ax = subplot(1,3,k,'Parent',f); hold(ax,'on'); axis(ax,'equal'); axis(ax,'off'); cleanAxes(ax);
        title(ax,objectNames{k},'FontSize',16,'FontWeight','normal');

        if k == 1
            th = linspace(0,2*pi,120);
            plot(ax,cos(th),sin(th),'k-','LineWidth',2);
            plot(ax,[0 0],[1 2.1],'-','LineWidth',3);
            plot(ax,[-1 -2],[-0.6 -1.2],'-','LineWidth',3);
            plot(ax,[1 2],[-0.6 -1.2],'-','LineWidth',3);
            contactText = 'point/curved contact';
        elseif k == 2
            th = linspace(0,2*pi,120);
            plot(ax,cos(th),sin(th),'k-','LineWidth',2);
            plot(ax,[-1 1],[1 1],'-','LineWidth',3);
            plot(ax,[-1 1],[-1 -1],'-','LineWidth',3);
            plot(ax,[0 0],[1 2.2],'-','LineWidth',3);
            plot(ax,[-1 -2],[-0.8 -1.3],'-','LineWidth',3);
            plot(ax,[1 2],[-0.8 -1.3],'-','LineWidth',3);
            contactText = 'line contact';
        else
            rectangle(ax,'Position',[-1 -1 2 2],'EdgeColor','k','LineWidth',2);
            plot(ax,[0 0],[1 2.2],'-','LineWidth',3);
            plot(ax,[-1 -2],[-0.7 -1.2],'-','LineWidth',3);
            plot(ax,[1 2],[-0.7 -1.2],'-','LineWidth',3);
            contactText = 'face/corner contact';
        end

        text(ax,-1.65,-1.55,sprintf('%s\\nangle %.1f%s',contactText,contactAngles(k),degSymbol),'FontSize',10);
        xlim(ax,[-2.7 2.7]); ylim(ax,[-1.9 2.5]);
    end
    safeSave(f,fullfile(figDir,'result_06_grasping_performance_objects.png'));

    %% 7. Before vs after comparison
    inputAngle = 0:25;
    oldDesign = min(36, 36*(1-exp(-inputAngle/7.5)) + 2*sin(inputAngle/4));
    newDesign = min(66, 66*(1-exp(-inputAngle/9)));

    f = figure('Name','Result 7 - Before vs after comparison','Color','w','Position',[100 80 950 560]);
    ax = axes('Parent',f); hold(ax,'on'); grid(ax,'on');
    plot(ax,inputAngle,oldDesign,'-s','LineWidth',2.2,'DisplayName','Old torsion-spring-only design');
    plot(ax,inputAngle,newDesign,'-o','LineWidth',2.2,'DisplayName','New contact-bar linkage design');
    title(ax,'Before vs after: fingertip bending response','FontSize',16,'FontWeight','normal');
    xlabel(ax,'Contact-bar/object input angle (degrees equivalent)');
    ylabel(ax,'Fingertip bending angle (degrees)');
    legend(ax,'Location','northwest');
    xlim(ax,[0 25]); ylim(ax,[-2 70]);
    cleanAxes(ax); safeSave(f,fullfile(figDir,'result_07_before_vs_after_comparison.png'));

    %% Extra graphs from the running video simulation
    f = figure('Name','Extra - Simulated finger bend','Color','w','Position',[100 80 920 560]);
    ax = axes('Parent',f); hold(ax,'on'); grid(ax,'on');
    plot(ax,history.closeRatio*100,'LineWidth',2);
    title(ax,'Simulation trace: STL finger bend during grip and release');
    xlabel(ax,'Frame'); ylabel(ax,'STL finger bend (%)');
    cleanAxes(ax); safeSave(f,fullfile(figDir,'extra_simulation_finger_bend_trace.png'));

    f = figure('Name','Extra - Grip force trace','Color','w','Position',[100 80 920 560]);
    ax = axes('Parent',f); hold(ax,'on'); grid(ax,'on');
    plot(ax,history.gripForce,'LineWidth',2);
    title(ax,'Simulation trace: total grip force');
    xlabel(ax,'Frame'); ylabel(ax,'Grip force (N)');
    cleanAxes(ax); safeSave(f,fullfile(figDir,'extra_simulation_grip_force_trace.png'));

    %% Summary text file
    summaryPath = fullfile(figDir,'assignment_results_summary.txt');
    fid = fopen(summaryPath,'w');
    if fid > 0
        fprintf(fid,'ASSIGNMENT RESULT MAP\\n');
        fprintf(fid,'=====================\\n\\n');
        fprintf(fid,'1. Kinematics of the finger mechanism -> result_01_kinematics_finger_mechanism.png\\n');
        fprintf(fid,'   Shows contact bar motion, linkage/pushrod motion, passive fingertip rotation, and fingertip path.\\n\\n');
        fprintf(fid,'2. Input-output relationship -> result_02_input_output_relationship.png\\n');
        fprintf(fid,'   Shows contact-bar displacement versus passive fingertip bending angle.\\n\\n');
        fprintf(fid,'3. Dimension optimization -> result_03_dimension_optimization.png\\n');
        fprintf(fid,'   Compares contact bar length, pushrod length, fingertip tab length, and final angle for D1-D5.\\n\\n');
        fprintf(fid,'4. Performance comparison -> result_04_performance_comparison.png\\n');
        fprintf(fid,'   Shows which tested dimensions give better gripping performance. D4 is best.\\n\\n');
        fprintf(fid,'5. Final optimized design -> result_05_final_optimized_design.png\\n');
        fprintf(fid,'   Presents selected dimensions: Lc = 25 mm, Lp = 19.69 mm, Lt = 21 mm.\\n\\n');
        fprintf(fid,'6. Grasping performance -> result_06_grasping_performance_objects.png\\n');
        fprintf(fid,'   Shows sphere, cylinder, and cube contact behavior with passive fingertip contact angles.\\n\\n');
        fprintf(fid,'7. Before vs after comparison -> result_07_before_vs_after_comparison.png\\n');
        fprintf(fid,'   Compares old torsion-spring-only design and new contact-bar linkage design.\\n');
        fclose(fid);
    end
end


function stlFile = findSTLFile(baseDir)
    names = {'Robotic_Picker.stl','Robotic Picker.stl','Robotic%20Picker.stl','full_robot_reference.stl'};
    for i = 1:numel(names)
        candidate = fullfile(baseDir,names{i});
        if exist(candidate,'file')
            stlFile = candidate;
            return;
        end
    end
    files = dir(fullfile(baseDir,'*.stl'));
    if ~isempty(files)
        stlFile = fullfile(baseDir,files(1).name);
        return;
    end
    error('No STL file found. Keep this .m file in the same folder as Robotic_Picker.stl.');
end

function [F,V] = readSTLCompat(file)
    try
        S = stlread(file);
        if isa(S,'triangulation')
            F = S.ConnectivityList; V = S.Points;
        elseif isstruct(S)
            if isfield(S,'faces') && isfield(S,'vertices')
                F = S.faces; V = S.vertices;
            elseif isfield(S,'ConnectivityList') && isfield(S,'Points')
                F = S.ConnectivityList; V = S.Points;
            else
                error('Unsupported stlread struct.');
            end
        else
            error('Unsupported stlread output.');
        end
    catch
        warning('stlread failed. Trying simple binary STL reader.');
        [F,V] = readBinarySTL(file);
    end
    F = double(F);
    V = double(V);
end

function [F,V] = readBinarySTL(file)
    fid = fopen(file,'rb');
    if fid < 0, error('Cannot open STL file.'); end
    cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fseek(fid,80,'bof');
    n = fread(fid,1,'uint32');
    V = zeros(3*n,3);
    F = zeros(n,3);
    for i = 1:n
        fread(fid,3,'single');
        pts = fread(fid,[3 3],'single')';
        fread(fid,1,'uint16');
        idx = (i-1)*3 + (1:3);
        V(idx,:) = pts;
        F(i,:) = idx;
    end
end

function R = rotzLocal(deg)
    R = [cosd(deg) -sind(deg) 0; sind(deg) cosd(deg) 0; 0 0 1];
end

function safeMkdir(d)
    if ~exist(d,'dir'), mkdir(d); end
end

function safeSave(fig,filePath)
    try
        saveas(fig,filePath);
    catch
        try
            print(fig,filePath,'-dpng','-r150');
        catch
        end
    end
end

function cleanAxes(ax)
    try
        ax.Toolbar.Visible = 'off';
    catch
    end
    try
        disableDefaultInteractivity(ax);
    catch
    end
end
