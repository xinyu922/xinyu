clc
clear all;
close all;
% 
rts_filename='RTS/RTS_ZED_20191105_1.csv';            %load rts csv file
camera_filename = 'sensor/20191106_zed_depth_1.csv';     %load camera file

%% ��ȡ�����ʱ��������ֵ
senser_struct=importdata(camera_filename);
sensor_time=senser_struct.data(:,1);         
sensor_depth=senser_struct.data(:,4);
z=find(sensor_depth==inf);
sensor_depth(z)=[];
sensor_time(z)=[];
%% ��ȡRTS��ʱ��������ֵ
rts_struct=importdata(rts_filename);
rts_struct.textdata(1,:)=[];
rts_struct.textdata(:,2)=[];

if rem(length(rts_struct.textdata),2)==1    %����ȥβ������ż����
    rts_struct.textdata=rts_struct.textdata(1:length(rts_struct.textdata)-1,:);
    rts_struct.data=rts_struct.data(1:length(rts_struct.data)-1,:);
end
rts_time2=[];
for i=1:length(rts_struct.textdata)                     %�ָ������������
    rts_time2(i,:)=str2double(cell2mat(rts_struct.textdata(i,:)));
end
rts_time=[];
for i=1:length(rts_time2)/2
    rts_time(i,1)=rts_time2(2*i-1,1);
end

rts_pic=[];
rts_2m=[];
rts_depth=[];
for i=1:length(rts_struct.textdata)                     %�ָ������������
    if(rem(i,2)==0)
        cow=size(rts_pic,1)+1;
        rts_pic(cow,1)=rts_struct.data(i,1);
        rts_pic(cow,2)=rts_struct.data(i,2);%
        rts_pic(cow,3)=rts_struct.data(i,3);%
    end 
    if(rem(i,2)==1)
        cow=size(rts_2m,1)+1; 
        rts_2m(cow,1)=rts_struct.data(i,1);
        rts_2m(cow,2)=rts_struct.data(i,2);
        rts_2m(cow,3)=rts_struct.data(i,3);
    end 
end
for i=1:size(rts_2m,1)
    rts_depth(i,1)=sqrt((rts_pic(i,1)-rts_2m(i,1))^2+(rts_pic(i,3)-rts_2m(i,3))^2+(rts_pic(i,2)-rts_2m(i,2))^2);
end

%%  ƥ��sensor��ʱ���RTS��ʱ��

k=1;
rts_t=[];
rts_d=[];
sensor_t=[];
sensor_d=[];
for i=1:length(sensor_time)
    for j=1:length(rts_time)
        if abs(rts_time(j,1)-sensor_time(i,1))<=0.008 %%Ĭ����Ϊ����ʱ���֮���ʱ����С��8ms,������ֵ�Ƕ�Ӧ��
            rts_t(k,1)=rts_time(j,1);
            rts_d(k,1)=rts_depth(j,1);
            sensor_t(k,1)=sensor_time(i,1);
            sensor_d(k,1)=sensor_depth(i,1);
            k=k+1;
            break;
        end
    end
    
end
%% plot    

figure;
subplot(2,2,1)
plot(sensor_t,sensor_d,'.');
title('Depth Measurement')
hold on;
plot(rts_t,rts_d,'.');
xlabel('Time(GPS)')%x����
ylabel('Measurement(m)')%y����
set(gca,'fontsize',12,'fontname','Times');
legend('sensor depth','rts depth'); 
grid on;
grid minor;
set(gca,'XMinorGrid','on')
set(gca,'Ytick',0:1:20) %���ü��    

subplot(2,2,2)
error=sensor_d-rts_d;
error_abs=abs(sensor_d-rts_d);
error_abs(isinf(error_abs)) = [];
% figure;
title('Error Disparity based on Distance') 
plot(rts_d,error_abs,':.');
xlabel('Ground Truth Distance(m)')%x����
ylabel('Error(m)')%y����
legend('error');
grid on;
grid minor;
set(gca,'fontsize',12,'fontname','Times');

subplot(2,2,3)
% figure;
title('Distance Based on GroundTruth') 
plot(rts_d,sensor_d,'.');
hold on;
plot(rts_d,rts_d,'.');
xlabel('Ground Truth(m)')%x����
ylabel('Distance Measured(m)')%y����
legend('Sensor Depth','Ground Truth');
grid on;
grid minor;
set(gca,'fontsize',12,'fontname','Times');

subplot(2,2,4)
% figure;
cdfplot(error_abs);
title('CDF');
xlabel('Error(m)')%x����
ylabel('Percentage(%)')%y����
legend('cdf');
grid on;
grid minor;
set(gca,'fontsize',12,'fontname','Times');


