version: 1.0
time: 2018-10-16

<ZED_get_capture>
<<获取原始图片信息，校正前和校正后的图形，采集后的数据自动创建4个文件夹进行保存>>

操作流程：
mkdir build/
cd build/
cmake ..
make
./zed_data_capture
