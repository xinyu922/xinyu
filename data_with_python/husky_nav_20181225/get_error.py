import pandas as pd
import re

filename = 'movebase_time.csv'
filename_husky = 'husky_poses.csv'
point_file = 'waypoint_ang.csv'


outfile = 'date_result.csv'

with open(filename, 'r') as f:
    rows = f.readlines()
    
df = pd.read_csv(filename)
point = pd.read_csv(point_file)
husky = pd.read_csv(filename_husky)
df = df[df.status == 3].reset_index(drop = True)

data = []
for i in range(len(df.time)):
    pose_temp = husky[(husky.timestamp > (df.time[i] - 0.1)) & (husky.timestamp < (df.time[i] + 0.1))]
    entry = pose_temp.mean().values
    columns_index = pose_temp.mean().index
    data.append(entry)
    
df_1 = pd.DataFrame(data, columns = columns_index)

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
df_1 = pd.DataFrame(pose, columns = list_1)  
df_2 = df_1.drop(['timestamp'], axis = 1)

erro = df_2 - point
erro['timestamp'] = df_1.timestamp
erro = erro.reindex(columns=list_1)
test = df_1.diff(1)

erro['time'] = test.timestamp
erro['dist'] = (test.x**2 + test.y**2)**0.5
erro['vel'] = erro['dist'] / erro['time']
erro.to_csv(outfile)
