clear all
close all
clc
[path] = uigetdir;
listing = dir(path);
load('stereo.mat')
cd(path)



for i=1:length(listing)
   if contains(listing(i).name, 'cam') && contains(listing(i).name, 'csv')
        opts = delimitedTextImportOptions("NumVariables", 3);
        opts.DataLines = [2, Inf];
        opts.Delimiter = ",";
        opts.VariableNames = ["index", "time", "imageName"];
        opts.VariableTypes = ["double", "double", "string"];
        opts = setvaropts(opts, 3, "WhitespaceRule", "preserve");
        opts = setvaropts(opts, 3, "EmptyFieldRule", "auto");
        opts.ExtraColumnsRule = "ignore";
        opts.EmptyLineRule = "read";
        eval(sprintf('%s = readtable(fullfile(listing(i).folder,listing(i).name), opts);', listing(i).name(1:end-4)));
   elseif contains(listing(i).name, 'imu')
        opts = delimitedTextImportOptions("NumVariables", 11);
        opts.DataLines = [2, Inf];
        opts.Delimiter = ",";
        opts.VariableNames = ["index", "time", "x", "y", "z", "ax", "ay", "az", "gx", "gy", "gz"];
        opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];
        opts.ExtraColumnsRule = "ignore";
        opts.EmptyLineRule = "read";
        eval(sprintf('%s = readtable(fullfile(listing(i).folder,listing(i).name), opts);', listing(i).name(1:end-4)));
   elseif contains(listing(i).name, 'radar')
        opts = delimitedTextImportOptions("NumVariables", 6);
        opts.DataLines = [2, Inf];
        opts.Delimiter = ",";
        opts.VariableNames = ["index", "time", "x", "y", "peakVal", "doppler"];
        opts.VariableTypes = ["double", "double", "string", "string", "string", "string"];
        opts = setvaropts(opts, [3, 4, 5, 6], "EmptyFieldRule", "auto");
        opts.ExtraColumnsRule = "ignore";
        opts.EmptyLineRule = "read";
        eval(sprintf('%s = readtable(fullfile(listing(i).folder,listing(i).name), opts);', listing(i).name(1:end-4)));
   end       
end
data_len = max([height(radar0), height(radar1), height(camera0), height(camera1)]);
'report'
['camera 0', 'camera1', 'radar0', 'radar1', 'imu0']
[sum(ismissing(camera0.imageName)), sum(ismissing(camera1.imageName)), sum(ismissing(radar0.x)), sum(ismissing(radar1.x)), sum(ismissing(imu0.ay))/5]/data_len*100
'number of samples:'
data_len
'duration:'
camera0.time(end)-camera0.time(1)
'mean sample rate'
data_len/(camera0.time(end)-camera0.time(1))


figure(1);
set(gcf, 'position',[0,0,900,400]);
data_len = min([height(radar0), height(radar1), height(camera0), height(camera1)]);

x=[];y=[];z=[];
R = @(t)([cosd(t), sind(t);-sind(t), cosd(t)]);
for i = 1:data_len
    if camera0{i,3}~="" && camera1{i,3}~=""
        I1= imread(camera0{i,3});
        I2=imread(camera1{i,3});
        subplot(2,3,1);imshow(I1);title('1-Left Camera')
        subplot(2,3,2);imshow(I2);title('2-Right Camera')
        [K1, K2] = rectifyStereoImages(I1, I2, stereoParams, 'OutputView', 'valid');
        J1 = rgb2gray(K1);
        J2 = rgb2gray(K2);
        disparityRange = [0 32];
        disparityMap = disparitySGM(J1,J2,'DisparityRange',disparityRange,'UniquenessThreshold',0);
        disparityMap_m = medfilt2(disparityMap,[3, 3]);
        f = ones(5,5)/25;
        disparityMap_f = filter2(f, disparityMap);
        out = (disparityMap_f + disparityMap_m)/2;
        subplot(2,3,3);imshow(out,disparityRange);title('3-Disparity Map')
        colormap(flipud(jet))
    end
    radar0{i,3} = replace(radar0{i,3},newline,'');
    radar0{i,4} = replace(radar0{i,4},newline,'');
    radar0{i,5} = replace(radar0{i,5},newline,'');
    radar0{i,6} = replace(radar0{i,6},newline,'');
    radar1{i,3} = replace(radar1{i,3},newline,'');
    radar1{i,4} = replace(radar1{i,4},newline,'');
    radar1{i,5} = replace(radar1{i,5},newline,'');
    radar1{i,6} = replace(radar1{i,6},newline,'');
    Pl = str2num(radar0{i,5});
    Dl = str2num(radar0{i,6});
    Xl = [str2num(radar0{i,3});str2num(radar0{i,4})];
    Pr = str2num(radar1{i,5});
    Dr = str2num(radar1{i,6});
    Xr = [str2num(radar1{i,3});str2num(radar1{i,4})];
    if ~isempty(Xl)
        Xl = R(25)*Xl + [.18;0];
    end
    if ~isempty(Xr)
        Xr = R(-25)*Xr + [-.18;0];
    end
     X = [Xl Xr];
    if isempty(X)
        X = [0 0;0 0];
        Pl= [0];Pr=[0];
        Dl= [0];Dr=[0];
    end
    [theta,rho] = cart2pol(X(1,:),X(2,:));
    subplot(2,3,4);
    polarplot(theta(1:length(Pl)),rho(1:length(Pl)),'.b',theta(length(Pl)+1:end),rho(length(Pl)+1:end),'.r', 'Markersize', 14);
    rlim([0,40]);thetalim([0 180]);title('4-RADAR (Left vs Right)')
    
    P = [Pl Pr]; T=10;
    subplot(2,3,5);
    polarplot(theta(P<=T),rho(P<=T),'.c',theta(P>T),rho(P>T),'.b', 'Markersize', 14);
    rlim([0,40]);thetalim([0 180]);title('5-RADAR (PeakVal)')
    
    D = [Dl Dr];
    subplot(2,3,6);
    polarplot(theta(D>0),rho(D>0),'.b',theta(D<0),rho(D<0),'.r',theta(D==0),rho(D==0),'.y', 'Markersize', 14);    
    rlim([0,40]);thetalim([0 180]);title('6-RADAR (Departing vs Approaching)')
    F(i) = getframe(gcf) ;
    drawnow
    if i >5
        break
    end
end

cd('..')
name=find(path=='\');
writerObj = VideoWriter([path(name(end)+1:end) '.avi']);
writerObj.FrameRate = 30;
open(writerObj);
% write the frames to the video
F = F(3:end);
for i=1:length(F)
    % convert the image to a frame
    frame = F(i) ;    
    writeVideo(writerObj, frame);
end
% close the writer object
close(writerObj);
close all

% 
% %%
% T=[];
% T(:,1) = radar0{1:5,2}; 
% T(:,2) = radar1{1:5,2};
% T(:,3) = leftcamera{1:5,2};
% T(:,4) = rightcamera{1:5,2};
% T(:,5) = imu{1:5,2};T=T(1:25);T=T-min(T);T=sort(T);
% figure(2);
% plot((T(9:13)-T(9))*1e6,'.', 'MarkerSize', 20);xlabel('sample');ylabel('\mu sec');grid on; xlim([0,6]);ylim([0,80])

    