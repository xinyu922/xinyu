import pandas as pd
import matplotlib.pylab as plt

filename = 'date_result.csv'
outfile = 'out_plot.csv'


df = pd.read_csv(filename)

df['sq_x_y'] = (df.x**2 + df.y**2)**0.5
df_1 = df.shift(1)  #数据向后移动一行，

data = {'point':[1,2,3,4,5,6,7,8,9]}  #创建新的dataframe
df_error = pd.DataFrame(data)

df_error['loop_1'] = df_1.sq_x_y[0:9].reset_index(drop = True)
df_error['loop_2'] = df_1.sq_x_y[9:18].reset_index(drop = True)
df_error['loop_3'] = df_1.sq_x_y[18:27].reset_index(drop = True)
df_error['loop_4'] = df_1.sq_x_y[27:36].reset_index(drop = True)
df_error['loop_5'] = df_1.sq_x_y[36:].reset_index(drop = True)

df_error['loop_1_yaw'] = df_1.yaw[0:9].reset_index(drop = True)
df_error['loop_2_yaw'] = df_1.yaw[9:18].reset_index(drop = True)
df_error['loop_3_yaw'] = df_1.yaw[18:27].reset_index(drop = True)
df_error['loop_4_yaw'] = df_1.yaw[27:36].reset_index(drop = True)
df_error['loop_5_yaw'] = df_1.yaw[36:].reset_index(drop = True)

df_error.to_csv(outfile, index = False)
df_error = abs(df_error)

index_ls = ['p1','p2','p3','p4','p5','p6','p7','p8','p9']

#画散点图，点的误差
# plt.subplot(2,1,1)
plt.plot(index_ls, df_error.loop_1, '.')
plt.plot(index_ls, df_error.loop_2, '*')
plt.plot(index_ls, df_error.loop_3, '<')
plt.plot(index_ls, df_error.loop_4, '^')
plt.plot(index_ls, df_error.loop_5, '>')

plt.xlabel('waypoints')
plt.ylabel('error/m')

plt.legend(loc = 'best')
plt.title('waypoints error for 5 loops')
plt.savefig('./point.jpg', dpi = 900, bbox_inches = 'tight')

plt.show()

#画散点图，角度的误差
# plt.subplot(2,1,2)
plt.plot(index_ls, df_error.loop_1_yaw, '.')
plt.plot(index_ls, df_error.loop_2_yaw, '*')
plt.plot(index_ls, df_error.loop_3_yaw, '>')
plt.plot(index_ls, df_error.loop_4_yaw, '^')
plt.plot(index_ls, df_error.loop_5_yaw, '<')

plt.xlabel('waypoints')
plt.ylabel('error/rad')
plt.legend(loc = 'best')
plt.title('angle error for 5 loops')

plt.savefig('./yaw.jpg', dpi = 900, bbox_inches = 'tight')

plt.savefig('./result.jpg', dpi = 1600, bbox_inches = 'tight')

plt.show()




