#!/usr/bin/python3
# -*- coding: UTF-8 -*-

import sys

import pandas as pd
import numpy as np
import matplotlib.pylab as plt

# zed
# rts_csv_path = 'D:\工作\测试项目\双目比测_20200421\RTSdata-20200421\dynamic\ZED_RTS_4m_20200421.csv'
# sensor_csv_path = 'D:\工作\测试项目\双目比测_20200421\sensor_data_20200421\dynamic\zed_depth_4m.csv'

# leadsense
# rts_csv_path = 'D:\工作\测试项目\双目比测_20200421\RTSdata-20200421\dynamic\LeadSense_RTS_4m_20200421.csv'
# sensor_csv_path = 'D:\工作\测试项目\双目比测_20200421\sensor_data_20200421\dynamic\leadSense_depth_4m.csv'

# mynt
rts_csv_path = 'D:\工作\测试项目\双目比测_20200421\RTSdata-20200421\dynamic\Mynt_RTS_4m_20200421.csv'
sensor_csv_path = 'D:\工作\测试项目\双目比测_20200421\sensor_data_20200421\dynamic\mynt_depth_4m.csv'

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

    # time = rts_pd['time'] - rts_pd.iloc[0, 0]
    time = rts_pd['time']
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
    # sensor_pd.drop(temp[abs(temp.v) > 30].index, inplace=True)

    # 获取sensor距离 时间
    time = sensor_pd['time'] + time_error
    time = time.tolist()
    distance = sensor_pd['depth'].tolist()

    return time, distance


def get_error(s_t, s_d, r_t, r_d):
    error = []
    r_d_inter = []

    for i in range(len(s_d)):
        for j in range(len(r_t)):
            if r_t[j] >= s_t[i]:
                l = r_t[j] - r_t[j - 1]
                l1 = s_t[i] - r_t[j - 1]
                d = (1 - l1 / l) * r_d[j - 1] + l1 / l * r_d[j]
                r_d_inter.append(d)
                break

    if len(s_d) > len(r_d_inter):
        s_d = s_d[:len(r_d_inter)]
    for i in range(len(s_d)):
        error.append(abs(r_d_inter[i] - s_d[i]))

    # 均值、方差计算
    error_mean = np.mean(error)
    error_std = np.std(error, ddof=1)
    print("error_mean: ", error_mean)
    print("error_std: ", error_std)
    return error


def plot_error(s_time, s_distance, r_time, r_distance):
    # error
    error = get_error(s_time, s_distance, r_time, r_distance)

    # 误差分布
    plt.subplot(321)
    plt.scatter(r_time, r_distance, color='r', s=2)
    plt.scatter(s_time, s_distance, color="b", s=2)
    plt.xlabel("Time")
    plt.ylabel('Distance')
    # plt.xlim([0, max(s_time)])
    # plt.title('Distance Measurement')
    plt.legend(['rts', 'sensor'])
    # plt.grid()

    # cdf
    plt.subplot(323)
    hist, bin_edges = np.histogram(error, bins=15)
    cdf = np.cumsum(hist / sum(hist))
    plt.xlabel("Error(m)")
    plt.ylabel('Percentage(%)')
    plt.xlim(0, np.max(bin_edges[1:]))
    plt.ylim(0, 1)

    ax = plt.gca()
    ax.xaxis.set_minor_locator(plt.MultipleLocator(0.05))  # 设置x从坐标间隔 0.1
    ax.yaxis.set_minor_locator(plt.MultipleLocator(0.05))  # 设置y从坐标间隔 0.1
    ax.grid(axis='x', which='minor')
    ax.grid(axis='y', which='minor')
    plt.grid()

    # plt.title('CDF')
    plt.plot(bin_edges[1:], cdf, '-*', color='#ED7D31')

    # hist
    plt.subplot(325)
    plt.xlabel("Error(m)")
    plt.ylabel('Nums')
    # plt.title('hist')
    plt.hist(error, bins=15)
    # plt.grid()

    # boxplot
    # plt.subplot(322)
    # plt.boxplot(error)

    # 分割字符串 存储照片
    img_name = rts_csv_path.split("\\")[-1].split('.')[0] + '.png'
    # plt.savefig(img_name, dpi=800)
    # np.save(rts_csv_path.split("\\")[-1].split('.')[0] + '.npy', error)

    plt.show()


def plot_box():

    zed_1m_error = np.load("ZED_RTS_1m_20200421.npy")
    zed_2m_error = np.load("ZED_RTS_2m_20200421.npy")
    zed_3m_error = np.load("ZED_RTS_3m_20200421.npy")
    zed_4m_error = np.load("ZED_RTS_4m_20200421.npy")
    zed_error = [zed_1m_error, zed_2m_error, zed_3m_error, zed_4m_error]

    leadsense_1m_error = np.load("LeadSense_RTS_1m_20200421.npy")
    leadsense_2m_error = np.load("LeadSense_RTS_2m_20200421.npy")
    leadsense_3m_error = np.load("LeadSense_RTS_3m_20200421.npy")
    leadsense_4m_error = np.load("LeadSense_RTS_4m_20200421.npy")
    leadsense_error = [leadsense_1m_error, leadsense_2m_error, leadsense_3m_error, leadsense_4m_error]

    mynt_1m_error = np.load("Mynt_RTS_1m_20200421.npy")
    mynt_2m_error = np.load("Mynt_RTS_2m_20200421.npy")
    mynt_3m_error = np.load("Mynt_RTS_3m_20200421.npy")
    mynt_4m_error = np.load("Mynt_RTS_4m_20200421.npy")
    mynt_error = [mynt_1m_error, mynt_2m_error, mynt_3m_error, mynt_4m_error]

    plt.figure(2)
    plt.boxplot(zed_error, showmeans=True)
    plt.title('zed_error: Box line diagram')
    plt.savefig('zed_error_box.png', dpi=800)
    plt.show()

    plt.figure(3)
    plt.boxplot(leadsense_error, showmeans=True)
    plt.title('leadsense_error: Box line diagram')
    plt.savefig('leadsense_error_box.png', dpi=800)
    plt.show()

    plt.figure(4)
    plt.boxplot(mynt_error, showmeans=True)
    plt.title('mynt_error: Box line diagram')
    plt.savefig('mynt_error_box.png', dpi=800)
    plt.show()

if __name__ == "__main__":
    rts_time, rts_distance = get_rts_data()
    sensor_time, sensor_distance = get_sensor_data()
    plot_error(sensor_time, sensor_distance, rts_time, rts_distance)
    plot_box()
