%%虚拟系统，双鱼眼相机平均值算法计算目标中心,实验测试
clear
% Laser Segmentation
name = "Matlab";
%a=Client;
Client = TCPInit('127.0.0.1',55014,name);
%Client=a;
image =ImageReadTCP_One(Client,'Center'); %imread('TestImages/image6.jpg');
image1 = ImageReadTCP_One1(Client,'Center');%imread('TestImages/image7.jpg');
%img = las_segm(image);
%img1 = las_segm(image1);
img = LaserFind(image);
img1 = LaserFind(image1);
% Configuration
load('Omni_Calib_Results_Unity.mat'); % Calib parameters
ocam_model = calib_data.ocam_model; % Calib parameters
camX =0;%-2.5; % Camera parameters 为z旋转
camY =0;%6; % Camera parameters 为x旋转
camZ =0;% 3; % Camera parameters 为y旋转
lasX = 0;%1.5; % Laser Plane parameters  
lasY = 0;%-2.5; % Laser Plane parameters
las_dist = 950; % Laser Plane parameters


camX1 =0;%-2.5; % Camera parameters 为z旋转
camY1 = 0;%6; % Camera parameters 为x旋转
camZ1 =0;% 3; % Camera parameters 为y旋转
lasX1 = 0;%1.5; % Laser Plane parameters
lasY1 = 0;%-2.5; % Laser Plane parameters
las_dist1 = 950; % Laser Plane parameters

CVsyst_x = -1700; % CV System initial position 在unity中为CVSystemOrigin的位置参数z*1000
CVsyst_y =-800; % CV System initial position 在unity中为CVSystemOrigin的位置参数x*-1000
CVsyst_rot = 0; % CV System initial rotation
CVsyst_x1 = 2500; % CV System second position 在unity中为CVSystemOrigin2的位置参数z*1000
CVsyst_y1 = 4500; % CV System second position 在unity中为CVSystemOrigin2的位置参数x*-1000
CVsyst_rot1 = 0;%20; % CV System second rotation 在unity中为CVSystemOrigin2的位置参数Rotation Y
% Mapping
[x,y] = mapping(img,CVsyst_rot,CVsyst_x,CVsyst_y,camX,camY,camZ,lasX,lasY,...
    las_dist,ocam_model); % mapping function
[x1,y1] = mapping(img1,CVsyst_rot1,CVsyst_x1,CVsyst_y1,camX1,camY1,camZ1,lasX1,...
    lasY1,las_dist1,ocam_model); % mapping function
% Finally figure:
figure;
scatter(x,y,5,'filled'); % Laser intersections, first image
hold on;
plot(CVsyst_x,CVsyst_y,'r*'); % CV System location, first image
scatter(x1,y1,5,'filled'); % Laser intersections, second image
plot(CVsyst_x1,CVsyst_y1,'r*'); % CV System location, second image
grid on;
%计算原始数据测距的平均值
XoriginMean=mean(x);
YoriginMean=mean(y);
XoriginMean1=mean(x1);
YoriginMean1=mean(y1);

%数据整合
combinedVector1 = [x, x1];
combinedVector2 = [y, y1];
combinedVector1 = nonzeros(combinedVector1);
combinedVector2 = nonzeros(combinedVector2);

combinedVector1(combinedVector1==CVsyst_x1)=[];
combinedVector2(combinedVector2==CVsyst_y1)=[];%去除向量内的原点坐标，减少干扰
combinedVector1(combinedVector1==CVsyst_x)=[];
combinedVector2(combinedVector2==CVsyst_y)=[];%去除向量内的原点坐标，减少干扰

%将激光矫正数据还原成图片
%Toimage1(combinedVector1,combinedVector2);
XcombinMean=mean(combinedVector1);
YcombinMean=mean(combinedVector2);

%x2=(max(combinedVector1)+min(combinedVector1))/2;
%y2=(max(combinedVector2)+min(combinedVector2))/2;

%寻找偏移角度
[Y1max,Y1maxRow]=max(combinedVector2);%寻找y最大值
X1value = combinedVector1(Y1maxRow);
[X3max,X3Row]=max(combinedVector1);%寻找x最大值
Y3value = combinedVector2(X3Row);
[X2min,X2Row]=min(combinedVector1);%寻找x最小值
Y2value = combinedVector2(X2Row);
[Y4min,Y4minRow]=min(combinedVector2);%寻找y最小值
X4value = combinedVector1(Y4minRow);
%寻找立方体中心点
L1=sqrt((X1value-X4value)^2+(Y1max-Y4min)^2);
L2=sqrt((X1value-X3max)^2+(Y1max-Y3value)^2);
L3=sqrt((X3max-X4value)^2+(Y3value-Y4min)^2);
thetha1=acosd((L1^2+L2^2-L3^2)/(2*L1*L2));
thetha2=thetha1-atan2d(X3max-X1value,Y1max-Y3value);%立方体中心点与y1点的偏角

test1=y1(10)-Y1max;%判断物体是否有偏移
if abs(test1)<100
    thetha6=0;
    x2=mean(combinedVector1);
    y2=mean(combinedVector2);
else
   thetha6=90-atan2d(X3max-X1value,Y1max-Y3value);
   x2=X1value-L1/2*sind(thetha2);
   y2=Y1max-L1/2*cosd(thetha2);

end
%绘制机器人坐标系图像
%POutput=[combinedVector1,combinedVector2];
%[potOld]=MappingRobotV(POutput);

POutput=[x;y];
POutput=reshape(POutput', [], 2);
[potOld]=MappingRobotV(POutput);

POutput1=[x1,y1];
POutput1=reshape(POutput1', [], 2);
[potOld1]=MappingRobotV(POutput1);

%相机坐标转换
CVsyst=PositionTran(CVsyst_x,CVsyst_y);
CVsyst1=PositionTran(CVsyst_x1,CVsyst_y1);
%LaserHigh=PositionTran(CVsyst_x1,CVsyst_y1);

%提取出各区域内线段坐标
% 按条件分类
Left = potOld(potOld(:,1) > 10 & potOld(:,1) < 15 & potOld(:,2) > -5 & potOld(:,2) < 10, :);
Right = potOld(potOld(:,1) > 30 & potOld(:,1) < 35 & potOld(:,2) > -5 & potOld(:,2) < 10, :);
Down = potOld(potOld(:,1) > 16 & potOld(:,1) < 27 & potOld(:,2) > -5 & potOld(:,2) < 0, :);
Up = potOld(potOld(:,1) > 16 & potOld(:,1) < 27 & potOld(:,2) > 10 & potOld(:,2) < 16, :);

% 按条件分类
Left1 = potOld1(potOld1(:,1) > 10 & potOld1(:,1) < 15 & potOld1(:,2) > -5 & potOld1(:,2) < 10, :);
Right1 = potOld1(potOld1(:,1) > 30 & potOld1(:,1) < 35 & potOld1(:,2) > -5 & potOld1(:,2) < 10, :);
Down1 = potOld1(potOld1(:,1) > 16 & potOld1(:,1) < 27 & potOld1(:,2) > -5 & potOld1(:,2) < 0, :);
Up1 = potOld1(potOld1(:,1) > 16 & potOld1(:,1) < 27 & potOld1(:,2) > 10 & potOld1(:,2) < 16, :);

%计算转换数据测距的平均值
XoriginMeanRLeft=mean(Left(:,1));%相机1数据
XoriginMeanRRight=mean(Right(:,1));
YoriginMeanRUp=mean(Up(:,2));
YoriginMeanRDown=mean(Down(:,2));


XoriginMeanR1Left1=mean(Left1(:,1));%相机2数据
XoriginMeanR1Right1=mean(Right1(:,1));
YoriginMeanRUp1=mean(Up1(:,2));
YoriginMeanRDown1=mean(Down1(:,2));


%数据整合
combinedVectorRLeft = [Left(:,1);Left1(:,1)];
combinedVectorRRight = [Right(:,1); Right1(:,1)];
combinedVectorRUp = [Up(:,2);Up1(:,2)];
combinedVectorRDown = [Down(:,2); Down1(:,2)];

XcombinMeanRLeft=mean(combinedVectorRLeft);
XcombinMeanRRight=mean(combinedVectorRRight);
YcombinMeanRUp=mean(combinedVectorRUp);
YcombinMeanRDown=mean(combinedVectorRDown);

%真实数据坐标
POutputRealLeft=PositionTran(-3900,220);
POutputRealRight=PositionTran(5010,220);
POutputRealUp=PositionTran(-40,5070);
POutputRealDown=PositionTran(-40,-2360);

%线段统一平均化
% 提取Left中的x和y坐标
combinedL = [Left; Left1];
x = combinedL(:, 1);
y = combinedL(:, 2);
% 选择拟合的多项式阶数（例如2次或3次）
polyOrder = 3; % 可根据需要调整阶数
% 进行多项式拟合
p = polyfit(y, x, polyOrder); % 注意：这里以y为自变量，x为因变量
% 生成拟合后的x值
yFit = unique(y); % 使用唯一的y值生成拟合点
xFit = polyval(p, yFit); % 计算拟合后的x值
% 将拟合后的数据存入FinalLeft
FinalLeft = [xFit, yFit];

% 提取Right中的x和y坐标
combinedR = [Right; Right1];
x = combinedR(:, 1);
y = combinedR(:, 2);
% 进行多项式拟合
p = polyfit(y, x, polyOrder); % 注意：这里以y为自变量，x为因变量
% 生成拟合后的x值
yFit = unique(y); % 使用唯一的y值生成拟合点
xFit = polyval(p, yFit); % 计算拟合后的x值
% 将拟合后的数据存入FinalLeft
FinalRight = [xFit, yFit];

% 提取Up中的x和y坐标
combinedU = [Up; Up1];
x = combinedU(:, 1);
y = combinedU(:, 2);
% 进行多项式拟合
p = polyfit(x, y, polyOrder); % 注意：这里以y为自变量，x为因变量
% 生成拟合后的x值
xFit = unique(x); % 使用唯一的y值生成拟合点
yFit = polyval(p, xFit); % 计算拟合后的x值
% 将拟合后的数据存入FinalLeft
FinalUp = [xFit, yFit];

% 提取Down中的x和y坐标
combinedD = [Down; Down1];
x = combinedD(:, 1);
y = combinedD(:, 2);
% 进行多项式拟合
p = polyfit(x, y, polyOrder); % 注意：这里以y为自变量，x为因变量
% 生成拟合后的x值
xFit = unique(x); % 使用唯一的y值生成拟合点
yFit = polyval(p, xFit); % 计算拟合后的x值
% 将拟合后的数据存入FinalLeft
FinalDown = [xFit, yFit];

figure;
scatter(FinalLeft(:,1),FinalLeft(:,2),5,'filled'); % Laser intersections, first image
hold on;
scatter(FinalRight(:,1),FinalRight(:,2),5,'filled'); % Laser intersections, first image
hold on;
scatter(FinalUp(:,1),FinalUp(:,2),5,'filled'); % Laser intersections, first image
hold on;
scatter(FinalDown(:,1),FinalDown(:,2),5,'filled'); % Laser intersections, first image
grid on;




% Finally figure:
figure;
scatter(potOld(:,1),potOld(:,2),5,'filled'); % Laser intersections, first image
hold on;
plot(CVsyst(1),CVsyst(2),'r*'); % CV System location, first image
scatter(potOld1(:,1),potOld1(:,2),5,'filled'); % Laser intersections, second image
plot(CVsyst1(1),CVsyst1(2),'r*'); % CV System location, second image
grid on;


%figure;
%scatter(xt2,yt2,5,'filled',"green");
%hold on;
%scatter(x2,y2,5,'filled',"blue"); % Laser intersections
%hold on;
%plot(potOld(1),potOld(2),'r*'); % CV System location
%plot(potOld(1),potOld(2),'g*');

%plot(20.15,6.61,'r*'); %真实物体中心位置，单位cm
%grid on;
%hold off;
% 设置图形的标题和轴标签
%title('Robot Coordinate System');
%xlabel('X/cm');
%ylabel('Y/cm');

output=[x2,y2,thetha6,];
