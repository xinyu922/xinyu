import pandas as pd
import math
import numpy as np
from matplotlib import pyplot as plt

rts_data_path = 'C:\\Users\\Administrator\\Desktop\\TrajectoryContrast\\depth_test\\20191105_depth_test\\RTS\\RTS_ZED_20191105_1.csv'
sensor_data_path = 'C:\\Users\\Administrator\\Desktop\\TrajectoryContrast\\depth_test\\20191105_depth_test\\sensor\\zed_depth20191105_1.csv'
rts_data = pd.DataFrame(pd.read_csv(rts_data_path))
sensor_data = pd.DataFrame(pd.read_csv(sensor_data_path))

rts_time_list = rts_data['time'].tolist()[::2]      # 获取rts_time_list

rts_data.drop(['time', 'name'], axis=1, inplace=True)
rts_data_diff = rts_data.diff()
rts_depth = rts_data_diff[rts_data_diff['x'] > 0].reset_index()
rts_depth['depth'] = (rts_depth['x']**2 + rts_depth['y']**2 + rts_depth['z']**2)**0.5

rts_depth_list = rts_depth['depth'].tolist()       # 获取rts_depth_list

if len(rts_time_list) > len(rts_depth_list):
    rts_time_list = rts_time_list[:len(rts_depth_list)]
else:
    rts_depth_list = rts_depth_list[:len(rts_time_list)]

plt.figure()
plt.scatter(rts_time_list, rts_depth_list)
plt.show()



# k = 1
# rts_t = []
# rts_d = []
# sensor_t = []
# sensor_d = []
#
# for i in len(sensor_data):
#     for j in len(rts_data):
#         if abs(rts_data['time'](j, 1) - sensor_data['time'](i, 1)) <= 0.008:
#             rts_t.append(rts_data['time'](j, 1))

