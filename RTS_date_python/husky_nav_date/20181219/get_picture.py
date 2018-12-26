import pandas as pd
import matplotlib.pylab as plt

filename = 'date_result.csv'
outfile = 'out_plot.csv'


df = pd.read_csv(filename)

df['sq_x_y'] = (df.x**2 + df.y**2)**0.5
df_1 = df.shift(1)  #数据向后移动一行

data = {'point':[1,2,3,4,5,6,7,8,9]}  #创建新的dataframe
df_erro = pd.DataFrame(data)

df_erro['loop_1'] = df_1.sq_x_y[0:9].reset_index(drop = True)
df_erro['loop_2'] = df_1.sq_x_y[9:18].reset_index(drop = True)
df_erro['loop_3'] = df_1.sq_x_y[18:27].reset_index(drop = True)
df_erro['loop_4'] = df_1.sq_x_y[27:36].reset_index(drop = True)
df_erro['loop_5'] = df_1.sq_x_y[36:].reset_index(drop = True)

df_erro['loop_1_yaw'] = df_1.yaw[0:9].reset_index(drop = True)
df_erro['loop_2_yaw'] = df_1.yaw[9:18].reset_index(drop = True)
df_erro['loop_3_yaw'] = df_1.yaw[18:27].reset_index(drop = True)
df_erro['loop_4_yaw'] = df_1.yaw[27:36].reset_index(drop = True)
df_erro['loop_5_yaw'] = df_1.yaw[36:].reset_index(drop = True)

df_erro.to_csv(outfile, index = False)

#画散点图，点的误差
plt.plot(df_erro.point, df_erro.loop_1, '.')
plt.plot(df_erro.point, df_erro.loop_2, '*')
plt.plot(df_erro.point, df_erro.loop_3, '<')
plt.plot(df_erro.point, df_erro.loop_4, '^')
plt.plot(df_erro.point, df_erro.loop_5, '>')

plt.xlabel('points')
plt.ylabel('erro/m')
plt.legend(loc = 'best')
plt.title('points erro for 5 loops')
plt.savefig('./point.jpg', dpi = 900, bbox_inches = 'tight')
plt.show()

#画散点图，角度的误差
plt.plot(df_erro.point, df_erro.loop_1_yaw, '.')
plt.plot(df_erro.point, df_erro.loop_2_yaw, '*')
plt.plot(df_erro.point, df_erro.loop_3_yaw, '>')
plt.plot(df_erro.point, df_erro.loop_4_yaw, '^')
plt.plot(df_erro.point, df_erro.loop_5_yaw, '<')

plt.xlabel('points')
plt.ylabel('erro/rad')
plt.legend(loc = 'best')
plt.title('angle erro for 5 loops')
plt.savefig('./yaw.jpg', dpi = 900, bbox_inches = 'tight')
plt.show()
