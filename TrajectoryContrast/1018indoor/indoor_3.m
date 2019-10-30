clc;
clear;
close all;
addpath('DATAS','FUNCTIONS');
take_off_stamp = 0.8;
%% load ground truth and estimate trajectory
G=importdata ('aft_mapped_to_init.txt');    % provided by EuRoC
aft_mappep_to_init=G.data;
cell_aft_mappep_t=G.textdata(2:size(G.textdata,1),1:1);
aft_mapped_t=zeros(size(cell_aft_mappep_t,1),1);
for i=1:size(cell_aft_mappep_t,1)
    aft_mapped_t(i,1)=str2double(cell2mat(cell_aft_mappep_t(i,1)))/1e9;
end
%�ȰѴ��µ������ƶ���ԭ��,��Ϊ����sim3DataAlignment��һ���߲���ģ��,����Ƴ߶ȣ�����ƽ������߶ȱ仯��ܴ�
GNGGA(:,1)=aft_mappep_to_init(:,1);
GNGGA(:,2)=aft_mappep_to_init(:,2);
GNGGA(:,3)=aft_mappep_to_init(:,3);

est=[aft_mapped_t,GNGGA];



litao=importdata ('litao.csv');
litao_data=litao.data;
cell_est_t=litao.textdata(2:size(litao.textdata,1),1:1);
litao_t=zeros(size(cell_est_t,1),1);
for i=1:size(cell_est_t,1)
    litao_t(i,1)=str2double(cell2mat(cell_est_t(i,1)));
end

gt=[litao_t,litao_data];
%% need to adjust the time standard of ground truth

gt(:,1) = gt(:,1)-take_off_stamp;

%% extract timestamp and 3d position from 'gt' and 'est', these are all we need for alignment
time_gt = gt(:,1);
P_gt = gt(:,2:4)';
time_es = est(:,1);
P_es = est(:,2:4)';

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
title('RTS��ֵ���Դ�������ںϵ�������Ա�','FontSize',15);
xlabel('x��,m')
ylabel('y��,m')
zlabel('z��,m')
%% draw error in XYZ axes
errX = P_es_aligned(1,Ids_es)-P_gt(1,Ids_gt);
errY = P_es_aligned(2,Ids_es)-P_gt(2,Ids_gt);
errZ = P_es_aligned(3,Ids_es)-P_gt(3,Ids_gt);
figure;
subplot(311);
plot(errX);
title('position error of axis-X (m)');
xlabel('������')
ylabel('��λm')
subplot(312);
plot(errY);
title('position error of axis-Y (m)');
xlabel('������')
ylabel('��λm')
subplot(313);
plot(errZ);
title('position error of axis-Z (m)');
xlabel('������')
ylabel('��λm')
%������仰�����mean error in [X Y Z]: [0.000000m 0.000000m
%0.000000m],������Ϊ��С����ֱ�ӰѲв�Ū�ķֲ��ܾ���
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
