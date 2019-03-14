# encoding: utf-8
import pandas as pd
import numpy as np
import re
import matplotlib.pylab as plt

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
# print(erro)
test = df_1.diff(1)

## 获取机器人运动轨迹的距离

husky_dis = husky[(husky.timestamp > erro.timestamp[0]) & (husky.timestamp < erro.timestamp[len(erro)-1])]
# husky_dis = husky_dis[husky_dis.timestamp < erro.timestamp[len(erro)-1]]
husky_dis = husky_dis.reset_index().drop('index', axis = 1)

list_1 = ['timestamp', 'x', 'y']
husky_dis = husky_dis.reindex(columns=list_1)

husky_sqr = husky_dis.diff(1).reset_index().drop(['index','timestamp'], axis = 1)

husky_sqr['sqr'] = (husky_sqr.x**2 + husky_sqr.y**2)**0.5
husky_sqr['timestamp'] = husky_dis.timestamp

real_dis = []
for i in range(len(erro)-1):
    temp_sqr = husky_sqr[(husky_sqr.timestamp > erro.timestamp[i]) & (husky_sqr.timestamp < erro.timestamp[i+1])]
    real_dis.append(temp_sqr.sqr.sum())
real_dis.insert(0,np.nan)

##

erro['time'] = test.timestamp
erro['dist'] = real_dis
erro['vel'] = erro['dist'] / erro['time']
erro.to_csv(outfile, index = False)


#获取9个位置点之间的实际距离（两点间的直线距离）
real_dis = point.append(point[0:1]).reset_index().drop(['index'], axis = 1)
real_dis = real_dis.diff(1).shift(-1)[0:9]
real_dis['dis'] = (real_dis.x**2 + real_dis.y**2)**0.5

#获取机器人运动过程总的时间，距离，速度
list_temp = ['timestamp', 'time', 'dist', 'vel']
data_robot = erro.reindex(columns=list_temp)
data_robot = data_robot.shift(-1)[:len(data_robot)-1]

data = {'point':[1,2,3,4,5,6,7,8,9]}  #创建新的dataframe
df_robot = pd.DataFrame(data)

#获取5个loop的时间信息
df_robot['time_1'] = data_robot.time[0:9].reset_index(drop = True)
df_robot['time_2'] = data_robot.time[9:18].reset_index(drop = True)
df_robot['time_3'] = data_robot.time[18:27].reset_index(drop = True)
df_robot['time_4'] = data_robot.time[27:36].reset_index(drop = True)
df_robot['time_5'] = data_robot.time[36:].reset_index(drop = True)

#获取5个loop的距离信息
df_robot['dist_1'] = data_robot.dist[0:9].reset_index(drop = True)
df_robot['dist_2'] = data_robot.dist[9:18].reset_index(drop = True)
df_robot['dist_3'] = data_robot.dist[18:27].reset_index(drop = True)
df_robot['dist_4'] = data_robot.dist[27:36].reset_index(drop = True)
df_robot['dist_5'] = data_robot.dist[36:].reset_index(drop = True)

#获取5个loop的速度信息
df_robot['vel_1'] = data_robot.vel[0:9].reset_index(drop = True)
df_robot['vel_2'] = data_robot.vel[9:18].reset_index(drop = True)
df_robot['vel_3'] = data_robot.vel[18:27].reset_index(drop = True)
df_robot['vel_4'] = data_robot.vel[27:36].reset_index(drop = True)
df_robot['vel_5'] = data_robot.vel[36:].reset_index(drop = True)

#画散点图，time
# plt.subplot(2,1,1)
plt.plot(df_robot.point, df_robot.time_1, '.')
plt.plot(df_robot.point, df_robot.time_2, '*')
plt.plot(df_robot.point, df_robot.time_3, '<')
plt.plot(df_robot.point, df_robot.time_4, '^')
plt.plot(df_robot.point, df_robot.time_5, '>')

plt.xlabel('points')
plt.ylabel('time')
plt.legend(loc = 'best')
plt.title('points time for 5 loops')
plt.savefig('./time.jpg', dpi = 900, bbox_inches = 'tight')

plt.show()

#画散点图，dist
# plt.subplot(2,1,1)
plt.plot(df_robot.point, df_robot.dist_1, '.')
plt.plot(df_robot.point, df_robot.dist_2, '*')
plt.plot(df_robot.point, df_robot.dist_3, '<')
plt.plot(df_robot.point, df_robot.dist_4, '^')
plt.plot(df_robot.point, df_robot.dist_5, '>')

#机器人的真实直线距离
plt.plot(df_robot.point, real_dis.dis, '*r')

plt.xlabel('points')
plt.ylabel('distance')
plt.legend(loc = 'best')
plt.title('points distance for 5 loops')
plt.savefig('./distance.jpg', dpi = 900, bbox_inches = 'tight')

plt.show()

#画散点图，dist
# plt.subplot(2,1,1)

df_robot.dist_1 = df_robot.dist_1/real_dis.dis
df_robot.dist_2 = df_robot.dist_2/real_dis.dis
df_robot.dist_3 = df_robot.dist_3/real_dis.dis
df_robot.dist_4 = df_robot.dist_4/real_dis.dis
df_robot.dist_5 = df_robot.dist_5/real_dis.dis

plt.plot(df_robot.point, df_robot.dist_1, '.')
plt.plot(df_robot.point, df_robot.dist_2, '*')
plt.plot(df_robot.point, df_robot.dist_3, '<')
plt.plot(df_robot.point, df_robot.dist_4, '^')
plt.plot(df_robot.point, df_robot.dist_5, '>')

#机器人的真实直线距离
# plt.plot(df_robot.point, real_dis.dis, '*r')

plt.xlabel('points')
plt.ylabel('%')
plt.legend(loc = 'best')
plt.title('points distance/dis for 5 loops')
plt.savefig('./distance_dis.jpg', dpi = 900, bbox_inches = 'tight')

plt.show()

#画散点图，velocity
# plt.subplot(2,1,1)
plt.plot(df_robot.point, df_robot.vel_1, '.')
plt.plot(df_robot.point, df_robot.vel_2, '*')
plt.plot(df_robot.point, df_robot.vel_3, '<')
plt.plot(df_robot.point, df_robot.vel_4, '^')
plt.plot(df_robot.point, df_robot.vel_5, '>')

plt.xlabel('points')
plt.ylabel('velocity')
plt.legend(loc = 'best')
plt.title('points velocity for 5 loops')
plt.savefig('./velocity.jpg', dpi = 900, bbox_inches = 'tight')

plt.show()
