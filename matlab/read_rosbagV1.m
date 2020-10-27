clear;

% filepath = fullfile('E:','BaiduNetdiskDownload','test_MYNT_color.bag');
filepath = fullfile('D:','工作','测试项目','lidar_camera','lidar_camera_1023_2020-10-23-13-55-32.bag');
bag = rosbag(filepath);

points_cloud = select(bag,'MessageType','sensor_msgs/PointCloud2');
%img= select(bag,'MessageType','sensor_msgs/Image');
img = select(bag,'Topic','/usb_cam/image_raw');
data = readMessages(points_cloud);
img_data = readMessages(img,1:10);
temp=reshape(img_data{1,1}.Data,3,1920,1080);
temp=permute(temp,[3,2,1]);

% temp = imread('depth_0.jpg');
% opencv 内参

fx = 4004.82116046;        
fy = 4014.34180153;
cx = 915.59121735;
cy = 448.48093506;

R_l2c = [  0.0109893, -0.999874, 0.0114295;
  0.0388102,  -0.0109951, -0.999186;
   0.999186,  0.0114239, 0.0386845 ];
t = [ -0.00874038, -0.544389, -0.650752];

% camera_leica 优化后内参
% fx = 3.80649015e+03;        
% fy = 3.80500975e+03;
% cx =  9.17973579e+02;
% cy = 4.56351056e+02;
% 
% R_l2c = [  0.0134712, -0.999843, 0.0114885;
%   0.0458134,  -0.0108603, -0.998891;
%    0.998859,  0.0139826, 0.0456599 ];
% t = [ -0.00800142, -0.55646, -0.706654];

R_in = [fx 0 cx;
    0 fy cy;
    0 0 1];

IntrinsicMatrix = [fx 0 0;
                   0 fy 0;
                   cx cy 1];
% radialDistortion = [-6.33120494e-01, 1.52092189e-01];
radialDistortion = [-0.55736824, 0.36627329];

% 相机去畸变

cameraParams = cameraParameters('IntrinsicMatrix', IntrinsicMatrix, 'RadialDistortion', radialDistortion);
temp = undistortImage(temp, cameraParams);

imshow(temp)


%imshow(temp);


frame = data{1,1};
xyz = readXYZ(frame);
intensity = readField(frame,'intensity');

x = xyz(:,1);
y = xyz(:,2);
z = xyz(:,3);
d = sqrt(x.*x + y.*y + z.*z);


% R = R_in * R_l2c;
xyz_size=size(xyz);
t = repmat(t, xyz_size(1), 1);
xyz = (R_l2c * xyz' + t')';
xyz = (R_in * xyz')';
x = xyz(:,1);
y = xyz(:,2);
z = xyz(:,3);
p_x = round(x ./ z);
p_y = round(y ./ z);




% figure(1);
% scatter3(p_x,p_y,d,'.');
% xlabel('X')
% ylabel('Y')
% zlabel('Z') 

figure(2);
%I0 = zeros(720,1280);
sel = p_x>0 & p_x<=1920 & p_y>0 & p_y <=1080 & z>0 ;
%sel = p_x<=0 | p_x>640 | p_y<=0 | p_y>480 | z<=0 ;
%d(sel) = 0;
%p_x(sel) = 1;
%p_y(sel) = 1;
new_px=p_x(sel);
new_py=p_y(sel);
temp(sub2ind(size(temp), new_py, new_px)) = 255;
temp(sub2ind(size(temp), min(1080,new_py+1), new_px)) = 255;
temp(sub2ind(size(temp), min(1080,new_py+1), min(1920,new_px+1))) = 255;
temp(sub2ind(size(temp), min(1080,new_py+1), max(1,new_px-1))) = 255;
temp(sub2ind(size(temp), new_py, max(1,new_px-1))) = 255;
temp(sub2ind(size(temp), new_py, min(1920,new_px+1))) = 255;
temp(sub2ind(size(temp), max(1,new_py-1), new_px)) = 255;
temp(sub2ind(size(temp), max(1,new_py-1), min(1920,new_px+1))) = 255;
temp(sub2ind(size(temp), max(1,new_py-1), max(1,new_px-1))) = 255;
imshow(temp)
%imshow(I0)
