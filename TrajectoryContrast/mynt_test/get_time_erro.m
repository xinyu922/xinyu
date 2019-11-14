clc;
clear;
close all;
addpath('DATAS','FUNCTIONS');

%% load ground truth and estimate trajectory
G=importdata ('vins_camera_pose.txt');    % provided by EuRoC
aft_mappep_to_init=G.data;
cell_aft_mappep_t=G.textdata(2:size(G.textdata,1),1:1);
aft_mapped_t=zeros(size(cell_aft_mappep_t,1),1);
for i=1:size(cell_aft_mappep_t,1)
    aft_mapped_t(i,1)=str2double(cell2mat(cell_aft_mappep_t(i,1)))/1e9;
end
%先把大致的坐标移动到原点,因为这里sim3DataAlignment是一个七参数模型,会估计尺度，不先平移坐标尺度变化会很大
GNGGA(:,1)=aft_mappep_to_init(:,1);
GNGGA(:,2)=aft_mappep_to_init(:,2);
GNGGA(:,3)=aft_mappep_to_init(:,3);

est=[aft_mapped_t,GNGGA];
EST=importdata ('RTS_pose.csv');
est_data=EST.data;
cell_est_t=EST.textdata(2:size(EST.textdata,1),1:1);
est_t=zeros(size(cell_est_t,1),1);
for i=1:size(cell_est_t,1)
    est_t(i,1)=str2double(cell2mat(cell_est_t(i,1)));
end

gt=[est_t,est_data];
%% need to adjust the time standard of ground truth
take_off_stamp=est(1,1)-gt(1,1)