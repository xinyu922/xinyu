import pandas as pd
import re

husky_pose_filename = 'husky_poses.csv'
point_pose_filename = 'waypoint_ang.csv'
timestamp_filename = 'move_base_timestamp.csv'
outfile = 'date_result.csv'

husky = pd.read_csv(husky_pose_filename)
point = pd.read_csv(point_pose_filename)
time = pd.read_csv(timestamp_filename)

f = open(outfile, 'w')

data = []
for i in range(len(time.timestamp)):
    pose_temp = husky[(husky.timestamp > (time.timestamp[i] - 0.2)) & (husky.timestamp < (time.timestamp[i] + 0.2))]
    entry = pose_temp.mean().values
    columns_index = pose_temp.mean().index
    data.append(entry)
df_1 = pd.DataFrame(data, columns = columns_index)
df_2 = df_1.drop(['timestamp'], axis = 1)

point_fix = (point.append(point)).reset_index().drop(['index'], axis = 1)
erro = df_2 - point_fix

test = df_1.diff(1)

erro['time'] = test.timestamp
erro['dist'] = (test.x**2 + test.y**2)**0.5
erro['vel'] = erro['dist'] / erro['time']

erro.to_csv(outfile)




    




