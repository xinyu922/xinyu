clc;
clear;
close all;
addpath('DATAS','FUNCTIONS');
husky=importdata('husky_poses.csv');
mynt=importdata('mynt_poses.csv');

husky_data=husky.data;
mynt_data=mynt.data;

time_gt = husky_data(:,1);
P_gt = husky_data(:,2:4)';
time_es = mynt_data(:,1);
P_es = mynt_data(:,2:4)';
%% alignment
%[Ids_es2, Ids_gt2] = findIds (time_es, time_gt, 1.0);
[Ids_gt2,Ids_es2 ] = findIds (time_gt, time_es, 0.0035);
if size(Ids_es2,1) < size(Ids_gt2,1)
        Ids_gt2(size(Ids_es2,1)+1:size(Ids_gt2,1),:)=[];
else
    if size(Ids_es2,1) > size(Ids_gt2,1)
        Ids_es2(size(Ids_gt2,1)+1:size(Ids_es2,1),:)=[];
    end
end
%Ids_es=Ids_es2(50:100,1:1);
%Ids_gt=Ids_gt2(50:100,1:1);
Ids_es=Ids_es2;
Ids_gt=Ids_gt2;
%[R_es, t_es, s_es] = sim3DataAlignment (P_es(:,Ids_es), P_gt(:,Ids_gt));
[R_es, t_es] = se3DataAlignment(P_es(:,Ids_es), P_gt(:,Ids_gt));
s_es=1;
%% do not miss '/s_es' to maintain the estimated scale
P_es_aligned = R_es*P_es + repmat(t_es,1,size(P_es,2))/s_es;

%% draw trajectory
time_matched = time_es(Ids_es);
P_es_matched = P_es_aligned(:,Ids_es);
P_gt_matched = P_gt(:,Ids_gt);
figure;
plot3(P_gt_matched(1,:),P_gt_matched(2,:),P_gt_matched(3,:),'r-');
hold on;
plot3(P_es_matched(1,:),P_es_matched(2,:),P_es_matched(3,:),'b-');
for i = 1:size(P_gt_matched,2)  % draw difference, this block is time consuming
    plot3([P_gt_matched(1,i),P_es_matched(1,i)],...
        [P_gt_matched(2,i),P_es_matched(2,i)],[P_gt_matched(3,i),P_es_matched(3,i)],'y-');
end
axis equal;
grid on;
lgd = legend('RTS','FUSION');
set(lgd,'Fontname','Times New Roman','FontWeight','bold','FontSize',15);
title('RTS真值与多源传感器融合导航结果对比','FontSize',15);
xlabel('x轴,m')
ylabel('y轴,m')
zlabel('z轴,m')
%% draw error in XYZ axes
errX = P_es_aligned(1,Ids_es)-P_gt(1,Ids_gt);
errY = P_es_aligned(2,Ids_es)-P_gt(2,Ids_gt);
errZ = P_es_aligned(3,Ids_es)-P_gt(3,Ids_gt);
figure;
subplot(311);
plot(errX);
title('position error of axis-X (m)');
xlabel('点序列')
ylabel('误差单位m')
subplot(312);
plot(errY);
title('position error of axis-Y (m)');
xlabel('点序列')
ylabel('误差单位m')
subplot(313);
plot(errZ);
title('position error of axis-Z (m)');
xlabel('点序列')
ylabel('误差单位m')
%下面这句话会输出mean error in [X Y Z]: [0.000000m 0.000000m
%0.000000m],这是因为最小二乘直接把残差弄的分布很均匀
fprintf('abs mean error in [X Y Z]: [%fm %fm %fm]\n',mean(abs(errX)),mean(abs(errY)),mean(abs(errZ)));
fprintf('std in [X Y Z distance]: [%fm %fm %fm %fm]\n',std(abs(errX)),std(abs(errY)),std(abs(errZ)),std(sqrt(abs(errX).^2+abs(errY).^2+abs(errZ).^2)));
%% some printing
errVec = P_es_aligned(1:3,Ids_es)-P_gt(1:3,Ids_gt);
N = size(errVec,2);
RMSE_trans = 0;
for i = 1:N
    RMSE_trans = RMSE_trans+norm(errVec(:,i))^2;
end
RMSE_trans = sqrt(RMSE_trans/N);
fprintf('RMSE of translation is %fm\n',RMSE_trans);
