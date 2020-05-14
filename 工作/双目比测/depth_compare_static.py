#!/usr/bin/python3
# -*- coding: UTF-8 -*-
import numpy as np
import pandas as pd
import matplotlib.pylab as plt

# zed
# rts_csv_path = 'D:\工作\测试项目\双目比测_20200421\RTSdata-20200421\static\ZED_RTS_14_20200421.csv'
# sensor_csv_path = 'D:\工作\测试项目\双目比测_20200421\sensor_data_20200421\static\zed_depth_14.csv'

# leadsense
# rts_csv_path = 'D:\工作\测试项目\双目比测_20200421\RTSdata-20200421\static\LeadSense_RTS_80_20200421.csv'
# sensor_csv_path = 'D:\工作\测试项目\双目比测_20200421\sensor_data_20200421\static\leadSense_depth_80.csv'

# mynt
rts_csv_path = 'D:\工作\测试项目\双目比测_20200421\RTSdata-20200421\static\Mynt_RTS_80_20200421.csv'
sensor_csv_path = 'D:\工作\测试项目\双目比测_20200421\sensor_data_20200421\static\mynt_depth_80.csv'

# 时间同步误差修正，默认为0
time_error = 0


def get_rts_data():
    rts_pd = pd.read_csv(rts_csv_path)
    # 去除无用的数据
    rts_pd.drop(['name', 'a', 'b', 'c', 'w'], axis=1, inplace=True)
    # 计算RTS距离 时间
    rts_diff_pd = rts_pd.diff(1)
    df_result = rts_diff_pd[rts_diff_pd.index % 2 == 1]
    df_result = df_result.reset_index(drop=True)
    df_result.drop('time', axis=1, inplace=True)
    df_result['distance'] = (df_result['x'] ** 2 + df_result['y'] ** 2 + df_result['z'] ** 2) ** 0.5
    distance = df_result['distance'].tolist()

    time = rts_pd['time'] - rts_pd.iloc[0, 0]
    time = time[time.index % 2 == 1].tolist()

    return time, distance


def get_sensor_data():
    sensor_pd = pd.read_csv(sensor_csv_path)

    # 处理distance中的异常值
    sensor_pd.drop(sensor_pd[sensor_pd['depth'] == np.inf].index, inplace=True)
    sensor_pd.drop(sensor_pd[sensor_pd['depth'] == 0].index, inplace=True)

    # 根据（u，v）的值剔除误识别的点
    temp = sensor_pd.diff()
    sensor_pd.drop(temp[abs(temp.u) > 50].index, inplace=True)
    sensor_pd.drop(temp[abs(temp.v) > 50].index, inplace=True)

    # 获取sensor距离 时间
    time = sensor_pd['time'] - sensor_pd.iloc[0, 0] + time_error
    time = time.tolist()
    distance = sensor_pd['depth'].tolist()

    return time, distance


def get_error(s_d, r_d):
    # error
    mean = sum(r_d)/len(r_d)
    error = []
    for i in range(len(s_d)):
        temp = s_d[i] - mean
        error.append(abs(temp))
    return error


def plot_error(s_time, s_distance, r_time, r_distance):

    error = get_error(s_distance, r_distance)
    # 误差分布
    plt.subplot(321)
    # plt.plot([r_time[0], r_time[-1]], [r_mean, r_mean], color='r')
    plt.plot(r_time, r_distance, color='r')
    plt.scatter(s_time, s_distance, color="b")
    plt.xlabel("Time")
    plt.ylabel('Distance')
    plt.xlim([0, max(s_time)])
    plt.legend(['rts', 'sensor'])

    # cdf
    plt.subplot(323)
    hist, bin_edges = np.histogram(error, bins=15)
    cdf = np.cumsum(hist / sum(hist))
    plt.xlabel("Error(m)")
    plt.ylabel('Percentage(%)')
    plt.plot(bin_edges[1:], cdf, '-*', color='#ED7D31')

    # hist
    plt.subplot(325)
    plt.xlabel("Error(m)")
    plt.ylabel('Nums')
    plt.hist(error, bins=15)

    # 分割字符串 存储照片
    img_name = sensor_csv_path.split("\\")[-1].split('.')[0] + '.png'
    plt.savefig(img_name, dpi=800)

    plt.show()


if __name__ == "__main__":
    rts_time, rts_distance = get_rts_data()
    sensor_time, sensor_distance = get_sensor_data()
    plot_error(sensor_time, sensor_distance, rts_time, rts_distance)











