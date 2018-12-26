import pandas as pd
import re

filename = 'movebase_time.csv'
filename_husky = 'husky_poses.csv'
point_file = 'waypoint.csv'
outfile = 'date_result.csv'
    
df = pd.read_csv(filename)
point = pd.read_csv(point_file)
husky = pd.read_csv(filename_husky)

df = df[df.status == 3].reset_index(drop = True)   #消除状态标志不是3的数据

#数据的筛选，取时间戳前后0.1秒的数据进行求和平均
data = []
for i in range(len(df.time)):
    pose_temp = husky[(husky.timestamp > (df.time[i] - 0.1)) & (husky.timestamp < (df.time[i] + 0.1))]
    entry = pose_temp.mean().values
    columns_index = pose_temp.mean().index
    data.append(entry)
    
df_1 = pd.DataFrame(data, columns = columns_index) 

#对获取的数据进行筛选，去到达目标点的第一个时刻的数据
temp_x = df_1.x[0]
temp_y = df_1.y[0]
pose = []
pose.append([df_1.timestamp[0], df_1.x[0], df_1.y[0], df_1.z[0], df_1.yaw[0], df_1.roll[0], df_1.pitch[0]])
for i in range(len(df_1.x) - 1):
    if ((df_1.x[i] - temp_x)**2 + (df_1.y[i] - temp_y)**2)**0.5 < 1:
        continue
    else:
        temp_x = df_1.x[i]
        temp_y = df_1.y[i]
        pose.append([df_1.timestamp[i], df_1.x[i], df_1.y[i], df_1.z[i], df_1.yaw[i], df_1.roll[i], df_1.pitch[i]])
pose
list_1 = ['timestamp', 'x','y','z','yaw','roll','pitch']
df_1 = pd.DataFrame(pose, columns = list_1)  #获取数据
df_2 = df_1.drop(['timestamp'], axis = 1)  #消除时间戳的信息，与waypoint.csv文件格式一致，用于对比

times = len(df_2)//len(point)
re = len(df_2)%len(point)

point_sum = point
for i in range(times - 1):
    point_sum = point_sum.append(point).reset_index().drop(['index'], axis = 1)
point_sum = point_sum.append(point[0:re]).reset_index().drop(['index'], axis = 1)

erro = df_2 - point_sum
erro['timestamp'] = df_1.timestamp
erro = erro.reindex(columns=list_1)
test = df_1.diff(1)

erro['time'] = test.timestamp
erro['dist'] = (test.x**2 + test.y**2)**0.5
erro['vel'] = erro['dist'] / erro['time']
erro.to_csv(outfile, index = False)
