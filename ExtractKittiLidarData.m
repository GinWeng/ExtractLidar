%This function is to stitch kitti lidar dataset
%written by Hang Zhao 
%20/10/2021 in Beijing
%

clc
clear
allpoints = [0,0,0];

% lidarpath = 'G:\downloads\2011_09_30_drive_0018_sync\2011_09_30\2011_09_30_drive_0018_sync\oxts\data';
lidarpath = 'F:\Capstone\algorithms\range_mcl\data(origin)\07\velodyne\';
gTruthpath = 'O:\My Drive\Study\80\Unimelb\capstone project\2021 New Capstone\Research Contents\Codes\2011_09_26_drive_0001_sync\oxts\data\';
% lidarpath = 'G:\downloads\2011_09_30_drive_0020_sync\2011_09_30\2011_09_30_drive_0020_sync\oxts\data';
listing = dir([gTruthpath, '\*.txt']);
nFrames = length(listing(not([listing.isdir])))

% recover x,y,z from gps
    for index=1:1:nFrames
        filename = [gTruthpath,num2str(index-1,'%010d'),'.txt'];
        
%         filename = ['G:\downloads\2011_09_30_drive_0020_sync\2011_09_30\2011_09_30_drive_0020_sync\oxts\data\ (',num2str(index),').txt'];
        gps = textread(filename);
        gps = gps(1:6);
        gps_flag = gps(1);
        gps(1) = gps(2);
        gps(2) = gps_flag;
        %lon,lat,alt,roll,pitch,yaw
        groundtruth(index,:) = gps;
        
    end

%using matlab toolbox
gt_xzy = zeros(nFrames,3);
    for i=1:1:nFrames
        lat = groundtruth(i,2);
        lon = groundtruth(i,1);
        alt = groundtruth(i,3);
        origin = [groundtruth(1,2),groundtruth(1,1),groundtruth(1,3)];
        [gt_xzy(i,1),gt_xzy(i,2),gt_xzy(i,3)] = latlon2local(lat,lon,alt,origin) ;
    end
  %gt_xzy = gt_xzy-gt_xzy(1,:);

% from vehicle coordinate system to lidar coordinate system
%rotate the coordinate system (2D)
% theta = -atan2(gt_xzy(3,2),gt_xzy(3,1));
% R = [cos(-theta) -sin(-theta); sin(-theta) cos(-theta)];
% gt_xzy(:,1:2) = (R*gt_xzy(:,1:2)')';

%stitch point cloud
interval = 1; 
% NUM=1;
gt = zeros(nFrames,12);
for fr=1:interval:nFrames
    fr
    %read lidar data
    lidarFileName = [lidarpath,num2str(fr-1,'%010d'),'.bin'];
    fid = fopen(lidarFileName);
    % fid = fopen(['G:\downloads\2011_09_30_drive_0020_sync\2011_09_30\2011_09_30_drive_0020_sync\velodyne_points\data\ (',num2str(fr),').bin']);
    
    A = fread(fid,'float');
    %pcshow(A(:,(1:3)));
    fclose(fid);

    pc = zeros(length(A)/4,3);
    for i=1:1:length(A)/4
        pc(i,1) = A((i-1)*4+1);
        pc(i,2)= A((i-1)*4+2);
        pc(i,3)= A((i-1)*4+3);
    end
    
    pcshow(pc);

    % drawnow
% save txt format
    % filename = ['lidarscan',num2str(NUM),'.txt'];
%     writematrix(pc,filename,'Delimiter','tab');
    % NUM=NUM+1;
    % figure(1)
    %  pcshow(pc);
    %  hold on

%extract rotation angle 
    gamma =groundtruth(fr,4); %roll

    beta = groundtruth(fr,5); %pitch
    alpha= groundtruth(fr,6);  %yaw
    eul = [alpha,beta,gamma];

% rotation matrix
    R = eul2rotm(eul);
    T = gt_xzy(fr,:);
    np = length(pc(:,1));
    pc = (R*pc')' + repmat(T,[np,1]);
%     figure(2)
%     pcshow(pc);
    %view(0,90)
%     hold on
%     drawnow
    
    %stitched point cloud
    allpoints = [allpoints;pc];
    gt(fr,1:3) = T+ones(1,3)*10e-6;
    gt(fr,4:6) = R(1,:);
    gt(fr,7:9) = R(2,:);
    gt(fr,10:12) = R(3,:);

end
writematrix(gt,'poses','Delimiter','tab');