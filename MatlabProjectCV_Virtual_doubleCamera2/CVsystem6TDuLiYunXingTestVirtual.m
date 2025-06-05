%虚拟系统，平均值算法计算目标中心
clear
% Laser Segmentation
name = "Matlab";
%a=Client;
Client = TCPInit('127.0.0.1',55014,name);
%Client=a;
image =ImageReadTCP_One1(Client,'Center'); %imread('TestImages/image6.jpg');


%image1 =snapshot(cam1);
%imwrite(image,'1.jpg');

%img1 = las_segm(image);
img=LaserFind(image);% LaserFind使用霍夫变换寻找直线，LaserFind2
%img1 = LaserFind3(image1);

% Mapping
load('Omni_Calib_Results_Unity.mat'); % Calib parameters
ocam_model = calib_data.ocam_model; % Calib parameters

camX =0;%-2.5; % Camera parameters 为z旋转
camY =0;%6; % Camera parameters 为x旋转
camZ =0;% 3; % Camera parameters 为y旋转
lasX = 0;%1.5; % Laser Plane parameters  
lasY = 0;%-2.5; % Laser Plane parameters
las_dist = 950; % Laser Plane parameters

CVsyst_x = -1700; % CV System initial position 在unity中为CVSystemOrigin的位置参数z*1000
CVsyst_y =-800; % CV System initial position 在unity中为CVSystemOrigin的位置参数x*-1000
CVsyst_rot = 0; % CV System initial rotation
% Mapping
[x,y] = mapping(img,CVsyst_rot,CVsyst_x,CVsyst_y,camX,camY,camZ,lasX,lasY,...
    las_dist,ocam_model); % mapping function
%[x1,y1] = mapping(img1,CVsyst_rot1,CVsyst_x1,CVsyst_y1,camX,camY,camZ,lasX,...
%    lasY,las_dist1,ocam_model); % mapping function
% Finally figure:
figure;
scatter(x,y,5,'filled'); % Laser intersections, first image
hold on;
plot(CVsyst_x,CVsyst_y,'r*'); % CV System location, first image
%scatter(x1,y1,5,'filled'); % Laser intersections, second image
%plot(CVsyst_x1,CVsyst_y1,'r*'); % CV System location, second image
grid on;


%绘制机器人坐标系图像
%POutput=[combinedVector1,combinedVector2];
%[potOld]=MappingRobotV(POutput);

POutput=[x;y];potOld=POutput;
%POutput=reshape(POutput', [], 2);
%[potOld]=MappingRobotR(POutput);

%POutput1=[x1,y1];
%POutput1=reshape(POutput1', [], 2);
%[potOld1]=MappingRobotR(POutput1);

%相机坐标转换
%CVsyst=PositionTranR2(CVsyst_x,CVsyst_y);
%CVsyst1=PositionTranR2(CVsyst_x1,CVsyst_y1);

% 合并两个数据变量
%mergedData = [potOld; potOld1];

% 提取满足条件的数据 (10 < x < 16)
%filteredData = mergedData(mergedData(:,1) > 10 & mergedData(:,1) < 16 & ...
%                          mergedData(:,2) > -10 & mergedData(:,2) < 10, :);
% 计算 x 和 y 的平均值
%averageX = mean(filteredData(:,1));
%averageY = mean(filteredData(:,2));



% Finally figure:
figure;
scatter(potOld(:,1),potOld(:,2),5,'filled'); % Laser intersections, first image
hold on;
%plot(CVsyst(1),CVsyst(2),'r*'); % CV System location, first image
%scatter(potOld1(:,1),potOld1(:,2),5,'filled'); % Laser intersections, second image
%plot(CVsyst1(1),CVsyst1(2),'r*'); % CV System location, second image
%plot(averageX,averageY,'r*');
grid on;



%output=[x2,y2,thetha6];
