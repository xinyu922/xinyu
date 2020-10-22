# -*- coding: utf-8 -*-
import numpy as np
import math
from scipy.optimize import leastsq
from scipy.optimize import minimize
from scipy.optimize import fsolve
import cv2
import os
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.colors import LogNorm
import Evaluate_depth


# #################-----------------获取棋盘格中位置点在像平面的坐标----------------#####################################
def distance_chessboard2camera_rt(rt, x, y, A, use_leastsq=True):
    temp_Rx = np.array(
        [[1, 0, 0, 0], [0, math.cos(rt[0]), -math.sin(rt[0]), 0], [0, math.sin(rt[0]), math.cos(rt[0]), 0],
         [0, 0, 0, 1]])
    temp_Ry = np.array(
        [[math.cos(rt[1]), 0, math.sin(rt[1]), 0], [0, 1, 0, 0], [-math.sin(rt[1]), 0, math.cos(rt[1]), 0],
         [0, 0, 0, 1]])
    temp_Rz = np.array(
        [[math.cos(rt[2]), -math.sin(rt[2]), 0, 0], [math.sin(rt[2]), math.cos(rt[2]), 0, 0], [0, 0, 1, 0],
         [0, 0, 0, 1]])
    temp_RT = np.dot(np.dot(temp_Rx, temp_Ry), temp_Rz)
    temp_RT[:, 3] = np.array([rt[3], rt[4], rt[5], 1]).reshape(1, 4)
    estimate_yc = np.dot(temp_RT, x)
    estimate_y = np.dot(A, estimate_yc)
    estimate_y /= estimate_yc[2]
    error = np.linalg.norm((estimate_y - y), ord=2, axis=0, keepdims=True)
    print(error.sum() / x.shape[1])
    if use_leastsq:
        return error[0]
    else:
        return error.sum()


def distance_chessboard2camera_rtA(rt, x, y, use_leastsq=True):
    temp_Rx = np.array(
        [[1, 0, 0, 0], [0, math.cos(rt[0]), -math.sin(rt[0]), 0], [0, math.sin(rt[0]), math.cos(rt[0]), 0],
         [0, 0, 0, 1]])
    temp_Ry = np.array(
        [[math.cos(rt[1]), 0, math.sin(rt[1]), 0], [0, 1, 0, 0], [-math.sin(rt[1]), 0, math.cos(rt[1]), 0],
         [0, 0, 0, 1]])
    temp_Rz = np.array(
        [[math.cos(rt[2]), -math.sin(rt[2]), 0, 0], [math.sin(rt[2]), math.cos(rt[2]), 0, 0], [0, 0, 1, 0],
         [0, 0, 0, 1]])
    temp_RT = np.dot(np.dot(temp_Rx, temp_Ry), temp_Rz)
    temp_RT[:, 3] = np.array([rt[3], rt[4], rt[5], 1]).reshape(1, 4)
    estimate_yc = np.dot(temp_RT, x)
    A = np.array([[rt[6], 0, rt[7], 0],
                  [0, rt[8], rt[9], 0],
                  [0, 0, 1, 0]])
    estimate_y = np.dot(A, estimate_yc)
    estimate_y /= estimate_yc[2]
    error = np.linalg.norm((estimate_y - y), ord=2, axis=0, keepdims=True)
    # print(error.sum() / x.shape[1])
    if use_leastsq:
        return error[0]
    else:
        return error.sum()


def get_cheesboard_3d_pos(img, internal_matrix, chessboard_xlen=0.03, chessboard_ylen=0.03, chessboard_xnum=11,
                          chessboard_ynum=8, is_colorful=True, cv2_input_type=True, opti_method="leastsq"):
    """
    :param img:input image with shape (H*W*3) or (3*H*W) if is colorful and is_colorful==True, (H*W) if is gray and is_colorful==False
    :param internal_matrix: the internal_matrix of camera with shape(3*3):[[fx,0,u0],
                                                                            [0,fy,v0],
                                                                            [0,0,1]]
                                                                    or (3,4)[[fx,0,u0,0],
                                                                            [0,fy,v0,0],
                                                                            [0,0,1,0]]
    :param chessboard_xlen: true length between each corner of the chessboard with x-axis direction,which unit is meter(m)
    :param chessboard_ylen: true length between each corner of the chessboard with y-axis direction,which unit is meter(m)
    :param chessboard_xnum: the numbers of corners on the chessboard with x-axis direction
    :param chessboard_ynum: the numbers of corners on the chessboard with y-axis direction
    :param is_colorful: is True when the input image is RGB image
    :param cv2_input_type: is True when the input image is read by opencv
    :param opti_method: the method of optimization now support leastsq and minimize from scipy.optimize
    :return:"str" if with some error during this function the return is the error info with the type of string
    else return "pos,rt_matrix", pos is the homogeneous coordinate of each corner with shape(4*corner_numbers)
                                rt_matrix is the rotation matrix and translate matrix from chessboard coordinate to camera coordinate with shape(4*4) [R|T]
    """
    if is_colorful:
        try:
            shape = img.shape
        except:
            return "【ERROR】: error with input image type"
        # if len(shape) == 4:

        if len(shape) != 3:
            return "【ERROR】: error with input image shape"
        if img.shape[-1] != 3:
            if img.shape[-1] == 4:
                img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
            else:
                try:
                    img = img.transpose((1, 2, 0))
                except:
                    return "【ERROR】: error with transpose input image shape"
        if not cv2_input_type:
            try:
                img = cv2.cvtColor(np.uint8(img), cv2.COLOR_RGB2BGR)
            except:
                return "【ERROR】: error with convert opencv type"
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img
    if chessboard_xnum <= 0:
        return "【ERROR】: error with input chessboard_xnum"
    if chessboard_ynum <= 0:
        return "【ERROR】: error with input chessboard_ynum"
    if chessboard_xlen <= 0:
        return "【ERROR】: error with input chessboard_xlen"
    if chessboard_ylen <= 0:
        return "【ERROR】: error with input chessboard_ylen"
    if not isinstance(internal_matrix, np.ndarray):
        internal_matrix = np.array(internal_matrix)
    if opti_method not in ["leastsq", "minimize", "cv2_solvePnP"]:
        return "【ERROR】: error with optimize method now support method is leastsq and minimize from scipy.optimize"
    try:
        shape = internal_matrix.shape
        if shape[0] == 3 and shape[1] == 3:
            internal_matrix = np.concatenate((internal_matrix, np.array([[0], [0], [0]])), axis=1)
        elif shape[0] != 3 and shape[1] != 4:
            return "【ERROR】: input internal matrix shape should be 3*4"
    except:
        return "【ERROR】: error with input image type"
    conernum = chessboard_xnum * chessboard_ynum
    criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)
    chessboard_corners = np.zeros((3, chessboard_xnum, chessboard_ynum))
    chessboard_corners[0:2, :, :] = np.mgrid[0:chessboard_xlen * chessboard_xnum:chessboard_xlen,
                                    0:chessboard_ylen * chessboard_ynum:chessboard_ylen]
    chessboard_corners_homo = np.ones((4, chessboard_xnum * chessboard_ynum))
    for i in range(chessboard_ynum):
        for j in range(chessboard_xnum):
            chessboard_corners_homo[0:3, i * chessboard_xnum + j] = chessboard_corners[:, j, i]
    ret, corners = cv2.findChessboardCorners(gray, (chessboard_xnum, chessboard_ynum))
    # cv2.imshow("gray",gray)
    # cv2.waitKey(0)
    if ret == False or corners.shape[0] != conernum:
        return "【ERROR】: without enough corners"
    corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)

    corners_homo = np.ones((3, chessboard_xnum * chessboard_ynum))
    corners_homo[0:2, :] = corners2[:, 0, :].T
    estimate_matrix = np.array([1, 1, 1, 0.1, 0.1, 0.1])
    if opti_method == "leastsq":
        rt = leastsq(distance_chessboard2camera_rt, estimate_matrix,
                     args=(chessboard_corners_homo, corners_homo, internal_matrix, True))
        temp_Rx = np.array(
            [[1, 0, 0, 0], [0, math.cos(rt[0][0]), -math.sin(rt[0][0]), 0],
             [0, math.sin(rt[0][0]), math.cos(rt[0][0]), 0],
             [0, 0, 0, 1]])
        temp_Ry = np.array(
            [[math.cos(rt[0][1]), 0, math.sin(rt[0][1]), 0], [0, 1, 0, 0],
             [-math.sin(rt[0][1]), 0, math.cos(rt[0][1]), 0],
             [0, 0, 0, 1]])
        temp_Rz = np.array(
            [[math.cos(rt[0][2]), -math.sin(rt[0][2]), 0, 0], [math.sin(rt[0][2]), math.cos(rt[0][2]), 0, 0],
             [0, 0, 1, 0],
             [0, 0, 0, 1]])
        temp_RT = np.dot(np.dot(temp_Rx, temp_Ry), temp_Rz)
        temp_RT[:, 3] = np.array([rt[0][3], rt[0][4], rt[0][5], 1]).reshape(1, 4)
    elif opti_method == "minimize":
        rt = minimize(distance_chessboard2camera_rt, estimate_matrix,
                      args=(chessboard_corners_homo, corners_homo, internal_matrix, False))
        rt = rt.x
        temp_Rx = np.array(
            [[1, 0, 0, 0], [0, math.cos(rt[0]), -math.sin(rt[0]), 0], [0, math.sin(rt[0]), math.cos(rt[0]), 0],
             [0, 0, 0, 1]])
        temp_Ry = np.array(
            [[math.cos(rt[1]), 0, math.sin(rt[1]), 0], [0, 1, 0, 0], [-math.sin(rt[1]), 0, math.cos(rt[1]), 0],
             [0, 0, 0, 1]])
        temp_Rz = np.array(
            [[math.cos(rt[2]), -math.sin(rt[2]), 0, 0], [math.sin(rt[2]), math.cos(rt[2]), 0, 0], [0, 0, 1, 0],
             [0, 0, 0, 1]])
        temp_RT = np.dot(np.dot(temp_Rx, temp_Ry), temp_Rz)
        temp_RT[:, 3] = np.array([rt[3], rt[4], rt[5], 1]).reshape(1, 4)
    else:
        _, r, t, inter = cv2.solvePnPRansac(chessboard_corners_homo[0:3, :].T, corners_homo[0:2, :].T,
                                            internal_matrix[:, 0:3], np.zeros((1, 5)))
        print("camera_pose", _)
        # temp_Rx = np.array(
        #     [[1, 0, 0, 0], [0, math.cos(r[0][0]), -math.sin(r[0][0]), 0], [0, math.sin(r[0][0]), math.cos(r[0][0]), 0],
        #      [0, 0, 0, 1]])
        # temp_Ry = np.array(
        #     [[math.cos(r[1][0]), 0, math.sin(r[1][0]), 0], [0, 1, 0, 0], [-math.sin(r[1][0]), 0, math.cos(r[1][0]), 0],
        #      [0, 0, 0, 1]])
        # temp_Rz = np.array(
        #     [[math.cos(r[2][0]), -math.sin(r[2][0]), 0, 0], [math.sin(r[2][0]), math.cos(r[2][0]), 0, 0], [0, 0, 1, 0],
        #      [0, 0, 0, 1]])
        rotation_m, _ = cv2.Rodrigues(r)
        # print(rotation_m)
        rt_matrix = np.concatenate((rotation_m, t), axis=1)
        temp_RT = np.concatenate((rt_matrix, np.array([[0, 0, 0, 1]])))
        # temp_RT = np.dot(np.dot(temp_Rx, temp_Ry), temp_Rz)
        # temp_RT[:, 3] = np.array([t[0][0], t[1][0], t[2][0], 1]).reshape(1, 4)
    pos = np.dot(temp_RT, chessboard_corners_homo)
    return pos, temp_RT


# #################----------------计算全站仪坐标到相机坐标的坐标转换-----------------#####################################
def distance_qzy2camera_rt(rt, x, y, use_leastsq=True):
    temp_Rx = np.array(
        [[1, 0, 0, 0], [0, math.cos(rt[0]), -math.sin(rt[0]), 0], [0, math.sin(rt[0]), math.cos(rt[0]), 0],
         [0, 0, 0, 1]])
    temp_Ry = np.array(
        [[math.cos(rt[1]), 0, math.sin(rt[1]), 0], [0, 1, 0, 0], [-math.sin(rt[1]), 0, math.cos(rt[1]), 0],
         [0, 0, 0, 1]])
    temp_Rz = np.array(
        [[math.cos(rt[2]), -math.sin(rt[2]), 0, 0], [math.sin(rt[2]), math.cos(rt[2]), 0, 0], [0, 0, 1, 0],
         [0, 0, 0, 1]])
    temp_RT = np.dot(np.dot(temp_Rx, temp_Ry), temp_Rz)
    temp_RT[:, 3] = np.array([rt[3], rt[4], rt[5], 1]).reshape(1, 4)
    estimate_y = np.dot(temp_RT, x)
    error = np.linalg.norm((estimate_y - y), ord=2, axis=0, keepdims=True)
    # print(error.sum()/x.shape[1])
    if use_leastsq:
        return error[0]
    else:
        # print(error.sum()/88)
        return error.sum()


def calibration_from_qzy2camera(qzy_pos_matrix, camera_pos_matrix, opti_method="leastsq"):
    """
    :param qzy_pos_matrix: 3d position in qzy coordinate with the shape (3*N) or (4*N)
    :param camera_pos_matrix: 3d position in camera coordinate with the shape (3*N) or (4*N)
    :param opti_method: the method of optimization now support leastsq and minimize from scipy.optimize
    :return: "str" if with some error during this function the return is the error info with the type of string
    else return "rt_matrix" rt_matrix is the rotation matrix and translate matrix from chessboard coordinate to camera coordinate with shape(4*4) [R|T]
    """
    if not isinstance(qzy_pos_matrix, np.ndarray):
        qzy_pos_matrix = np.array(qzy_pos_matrix)
    if not isinstance(camera_pos_matrix, np.ndarray):
        camera_pos_matrix = np.array(camera_pos_matrix)
    if len(qzy_pos_matrix.shape) != 2:
        return "【ERROR】: error with input qzy_pos_matrix shape"
    if qzy_pos_matrix.shape[0] == 3:
        qzy_pos_matrix = np.concatenate((qzy_pos_matrix, np.ones((1, qzy_pos_matrix.shape[1]))))
    elif qzy_pos_matrix.shape[0] != 4:
        return "【ERROR】: error with input qzy_pos_matrix shape"
    if len(camera_pos_matrix.shape) != 2:
        return "【ERROR】: error with input camera_pos_matrix shape"
    if camera_pos_matrix.shape[0] == 3:
        camera_pos_matrix = np.concatenate((camera_pos_matrix, np.ones((1, camera_pos_matrix.shape[1]))))
    elif camera_pos_matrix.shape[0] != 4:
        return "【ERROR】: error with input camera_pos_matrix shape"
    if qzy_pos_matrix.shape[1] != camera_pos_matrix.shape[1]:
        return "【ERROR】: the shape of qzy_pos_matrix and camera_pos_matrix should be same"
    if opti_method not in ["leastsq", "minimize"]:
        return "【ERROR】: error with optimize method now support method is leastsq and minimize from scipy.optimize"
    estimate_matrix = np.array([1.57, 0, -1.57, 0, 0, 0])
    if opti_method == "leastsq":
        rt = leastsq(distance_qzy2camera_rt, estimate_matrix, args=(qzy_pos_matrix, camera_pos_matrix, True),
                     maxfev=500000)
        temp_Rx = np.array(
            [[1, 0, 0, 0], [0, math.cos(rt[0][0]), -math.sin(rt[0][0]), 0],
             [0, math.sin(rt[0][0]), math.cos(rt[0][0]), 0],
             [0, 0, 0, 1]])
        temp_Ry = np.array(
            [[math.cos(rt[0][1]), 0, math.sin(rt[0][1]), 0], [0, 1, 0, 0],
             [-math.sin(rt[0][1]), 0, math.cos(rt[0][1]), 0],
             [0, 0, 0, 1]])
        temp_Rz = np.array(
            [[math.cos(rt[0][2]), -math.sin(rt[0][2]), 0, 0], [math.sin(rt[0][2]), math.cos(rt[0][2]), 0, 0],
             [0, 0, 1, 0],
             [0, 0, 0, 1]])
        temp_RT = np.dot(np.dot(temp_Rx, temp_Ry), temp_Rz)
        temp_RT[:, 3] = np.array([rt[0][3], rt[0][4], rt[0][5], 1]).reshape(1, 4)
    else:
        rt = minimize(distance_qzy2camera_rt, estimate_matrix, args=(qzy_pos_matrix, camera_pos_matrix, False))
        print(rt.success)
        rt = rt.x
        temp_Rx = np.array(
            [[1, 0, 0, 0], [0, math.cos(rt[0]), -math.sin(rt[0]), 0], [0, math.sin(rt[0]), math.cos(rt[0]), 0],
             [0, 0, 0, 1]])
        temp_Ry = np.array(
            [[math.cos(rt[1]), 0, math.sin(rt[1]), 0], [0, 1, 0, 0], [-math.sin(rt[1]), 0, math.cos(rt[1]), 0],
             [0, 0, 0, 1]])
        temp_Rz = np.array(
            [[math.cos(rt[2]), -math.sin(rt[2]), 0, 0], [math.sin(rt[2]), math.cos(rt[2]), 0, 0], [0, 0, 1, 0],
             [0, 0, 0, 1]])
        temp_RT = np.dot(np.dot(temp_Rx, temp_Ry), temp_Rz)
        temp_RT[:, 3] = np.array([rt[3], rt[4], rt[5], 1]).reshape(1, 4)
    return temp_RT, rt


# #################----------------全站仪点云投影到相机的像平面，获取RGB图及depth图-----------------#####################################
def project_pointcloud_to_image_four_dimension(point_cloud_matrix, internal_matrix, image_size):
    """
    :param point_cloud_matrix: point cloud from qzy with shape(3,N) or (4,N) without RGB value; shape(6,N) or (7,N) with RGB value
    :param internal_matrix: internal matrix of camera with shape (3,3) or (3,4) or (5,6) or (6,7)
                                                                                            [f 0 u 0 0 0 0           [f 0 u 0 0 0 0
                                                                                             0 f v 0 0 0 0            0 f v 0 0 0 0
                                                                                             0 0 1 0 0 0 0     OR     0 0 0 1 0 0 0
                                                                                             0 0 0 0 1 0 0            0 0 0 0 1 0 0
                                                                                             0 0 0 0 0 1 0            0 0 0 0 0 1 0
                                                                                             0 0 0 0 0 0 1]           0 0 0 0 0 0 1]
    :param image_size: output images size with W*H
    :return: "str" if with some error during this function the return is the error info with the type of string
    else return "depth_matrix ,RGB_matrix" depth matrix with shape W*H and default value -1 RGB_matrix with shape 3*W*H and default value 0
    """
    if not isinstance(point_cloud_matrix, np.ndarray):
        point_cloud_matrix = np.array(point_cloud_matrix)
    if not isinstance(internal_matrix, np.ndarray):
        internal_matrix = np.array(internal_matrix)
    if len(image_size) != 2:
        return "【ERROR】the length of image_size should be 2,with (W,H)"
    if len(point_cloud_matrix.shape) != 2:
        return "【ERROR】error with the input point_cloud_matrix shape"
    if len(internal_matrix.shape) != 2:
        return "【ERROR】error with the input internal_matrix shape"
    if point_cloud_matrix.shape[1] < point_cloud_matrix.shape[0]:
        point_cloud_matrix = point_cloud_matrix.T
    if point_cloud_matrix.shape[0] == 3:
        point_cloud_matrix = np.concatenate((point_cloud_matrix, np.ones((1, point_cloud_matrix.shape[1]))))
    elif point_cloud_matrix.shape[0] == 6:
        point_cloud_matrix = np.concatenate(
            (point_cloud_matrix[0:3], np.ones((1, point_cloud_matrix.shape[1])), point_cloud_matrix[3:6]))
    elif point_cloud_matrix.shape[0] != 4 and point_cloud_matrix.shape[0] != 7:
        return "【ERROR】error with the input point_cloud_matrix shape"
    if point_cloud_matrix.shape[0] == 4:
        if internal_matrix.shape[0] == 3 and internal_matrix.shape[1] == 3:
            internal_matrix = np.concatenate((internal_matrix, np.array([[0], [0], [0]])), axis=1)
        elif internal_matrix.shape[0] != 3 and internal_matrix.shape[1] != 4:
            return "【ERROR】: input internal matrix shape should be 3*4 without RGB"
    else:
        if internal_matrix.shape[0] == 3 and internal_matrix.shape[1] == 3:
            internal_matrix = np.concatenate((internal_matrix, np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]])),
                                             axis=1)
            internal_matrix = np.concatenate(
                (internal_matrix, np.array([[0, 0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 0, 1]])))
        elif internal_matrix.shape[0] == 3 and internal_matrix.shape[1] == 4:
            internal_matrix = np.concatenate((internal_matrix, np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])), axis=1)
            internal_matrix = np.concatenate(
                (internal_matrix, np.array([[0, 0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 0, 1]])))
        elif internal_matrix.shape[0] == 5 and internal_matrix.shape[1] == 6:
            internal_matrix = np.concatenate(
                (internal_matrix[0:2], np.array([[0, 0, 1, 0, 0, 0]]), internal_matrix[2:5]))
            internal_matrix = np.concatenate((internal_matrix[:, 0:3], np.zeros((6, 1)), internal_matrix[:, 3:6]),
                                             axis=1)
        elif internal_matrix.shape[0] != 6 and internal_matrix[1] != 7:
            return "【ERROR】: input internal matrix shape should be 6*7 with RGB"
    RGB_flag = True
    if point_cloud_matrix.shape[0] == 4:
        RGB_flag = False
    internal_matrix[2, 2] = 0
    internal_matrix[2, 3] = 1
    point_cloud_matrix[0] /= point_cloud_matrix[2]
    point_cloud_matrix[1] /= point_cloud_matrix[2]
    point_cloud_matrix[3] = point_cloud_matrix[2]
    point_cloud_matrix[2] /= point_cloud_matrix[2]
    uvd_rgb = np.dot(internal_matrix, point_cloud_matrix)
    uv_point_info_dict = {}
    for i in range(len(uvd_rgb[0])):
        temp_u = uvd_rgb[0][i]
        temp_v = uvd_rgb[1][i]
        temp_z = uvd_rgb[2][i]
        if temp_u >= 0 and temp_u <= image_size[0] and temp_v >= 0 and temp_v <= image_size[1] and temp_z > 0:
            temp_u = int(temp_u)
            temp_v = int(temp_v)
            if "{0},{1}".format(temp_u, temp_v) in uv_point_info_dict.keys():
                uv_point_info_dict["{0},{1}".format(temp_u, temp_v)].append(uvd_rgb[:, i])
            else:
                uv_point_info_dict["{0},{1}".format(temp_u, temp_v)] = [uvd_rgb[:, i]]
    depth_matrix = np.ones((image_size[0], image_size[1])) * -1
    # print(uv_point_info_dict["724,391"])
    if RGB_flag:
        RGB_matrix = np.zeros((3, image_size[0], image_size[1]))
    for key, value in uv_point_info_dict.items():
        point_index = key.split(',')
        temp_u = int(point_index[0])
        temp_v = int(point_index[1])
        if len(value) == 1:
            depth_matrix[temp_u, temp_v] = value[0][2]
            if RGB_flag:
                RGB_matrix[0, temp_u, temp_v] = value[0][3]
                RGB_matrix[1, temp_u, temp_v] = value[0][4]
                RGB_matrix[2, temp_u, temp_v] = value[0][5]
        else:
            temp_array = np.array(value)
            # distance = np.sqrt((temp_array[:, 0] - temp_u) ** 2 + (temp_array[:, 1] - temp_v) ** 2)
            distance = temp_array[:, 2]
            if max(temp_array[:, 2]) - min(temp_array[:, 2]) > 0.05:
                index = np.argmin(distance)
                depth_matrix[temp_u, temp_v] = value[index][2]
                if RGB_flag:
                    RGB_matrix[0, temp_u, temp_v] = value[index][3]
                    RGB_matrix[1, temp_u, temp_v] = value[index][4]
                    RGB_matrix[2, temp_u, temp_v] = value[index][5]
            else:
                distance = np.sqrt((temp_array[:, 0] - temp_u) ** 2 + (temp_array[:, 1] - temp_v) ** 2)
                temp_distance = 1 / distance
                sum_distance = temp_distance.sum()
                weight = temp_distance / sum_distance
                weighted_value = np.dot(weight, temp_array)
                depth_matrix[temp_u, temp_v] = weighted_value[2]
                if RGB_flag:
                    RGB_matrix[0, temp_u, temp_v] = weighted_value[3]
                    RGB_matrix[1, temp_u, temp_v] = weighted_value[4]
                    RGB_matrix[2, temp_u, temp_v] = weighted_value[5]
    if RGB_flag:
        return depth_matrix, RGB_matrix
    else:
        return depth_matrix, None


def get_camera_internal_reference(name):
    # zed相机内参

    # ros
    # zed = np.array([[683.9474, 0, 679.673, 0],
    #                 [0, 683.6081, 369.784, 0],
    #                 [0, 0, 1, 0]])

    # calirarion
    zed = np.array([[672.316, 0, 677.478, 0],
                    [0, 675.169, 370.290, 0],
                    [0, 0, 1, 0]])

    # leadsense相机内参

    # ros
    # leadsense = np.array([[708.881, 0, 634.701, 0],
    #                       [0, 708.881, 331.160, 0],
    #                       [0, 0, 1, 0]])

    # calibration
    leadsense = np.array([[708.616, 0, 625.578, 0],
                          [0, 711.519, 336.317, 0],
                          [0, 0, 1, 0]])
    # mynt相机内参
    # ros
    mynt = np.array([[708.183, 0, 635.238, 0],
                     [0, 709.342, 362.509, 0],
                     [0, 0, 1, 0]])

    # calibration
    # mynt = np.array([[707.311, 0, 625.271, 0],
    #                  [0, 709.841, 362.910, 0],
    #                  [0, 0, 1, 0]])

    if name == 'zed':
        return zed
    elif name == 'leadsense':
        return leadsense
    elif name == 'mynt':
        return mynt
    else:
        print("【ERROR】: 你输入的相机不符合要求！！！, 请查看函数get_camera_internal_reference的输入参数！")


def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        if param == 'zed':
            depth_data = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\zed\20200514\result\depth_matrix.npy')
            camera_data = np.genfromtxt(r'D:\工作\测试项目\深度图_全站仪\camera_data\20200515\zed_depth\zed_depth_0.csv',delimiter=',')
            factor = 1

            # qzy 深度图
            qzy_depth_data = depth_data
            qzy_depth_data_min = 0
            qzy_depth_data_max = qzy_depth_data.max()
            qzy_depth_data_invalid = np.where(qzy_depth_data < 0)
            qzy_depth_data[qzy_depth_data_invalid] = 0
            qzy_depth_data = (qzy_depth_data - qzy_depth_data_min) / (qzy_depth_data_max - qzy_depth_data_min) * 255
            qzy_depth_data = qzy_depth_data.astype(np.uint8).T
            cv2.imshow('qzy_zed_depth', qzy_depth_data)
            # cv2.imwrite('qzy_zed_depth.png', qzy_depth_data)

            # zed 深度图
            camera_depth_data = camera_data
            camera_depth_data_min = 0
            camera_depth_data_invalid = np.where(np.isnan(camera_depth_data))
            camera_depth_data[camera_depth_data_invalid] = 0
            camera_depth_data_max = camera_depth_data.max()
            camera_depth_data = (camera_depth_data - camera_depth_data_min) / (
                        camera_depth_data_max - camera_depth_data_min) * 255
            camera_depth_data = camera_depth_data.astype(np.uint8)
            cv2.imshow('zed_depth', camera_depth_data)
            # cv2.imwrite('zed_depth.png', camera_depth_data)

        elif param == 'leadsense':
            depth_data = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\leadsense\20200520\result\depth_matrix.npy')
            camera_data = np.genfromtxt(r'D:\工作\测试项目\深度图_全站仪\camera_data\202005220\leadsense_depth_20200520\leadsense_depth\leadSense_depth_1.csv',
                delimiter=',')
            factor = 1000

            # qzy深度图
            qzy_depth_data = depth_data
            qzy_depth_data_min = 0
            qzy_depth_data_max = qzy_depth_data.max()
            qzy_depth_data_invalid = np.where(qzy_depth_data < 0)
            qzy_depth_data[qzy_depth_data_invalid] = 0
            qzy_depth_data = (qzy_depth_data - qzy_depth_data_min) / (qzy_depth_data_max - qzy_depth_data_min) * 255
            qzy_depth_data = qzy_depth_data.astype(np.uint8).T
            cv2.imshow('qzy_leadsenes_depth', qzy_depth_data)
            # cv2.imwrite('qzy_leadsenes_depth.png', qzy_depth_data)

            # leadsense 深度图
            camera_depth_data = camera_data[:, 0:1280]
            camera_depth_data_min = 0
            camera_depth_data_invalid = np.where(np.isinf(camera_depth_data))
            camera_depth_data[camera_depth_data_invalid] = 0
            camera_depth_data_max = camera_depth_data.max()
            camera_depth_data = (camera_depth_data - camera_depth_data_min) / (camera_depth_data_max - camera_depth_data_min) * 255
            camera_depth_data = camera_depth_data.astype(np.uint8)
            cv2.imshow('leadsense', camera_depth_data)
            # cv2.imwrite('leadsense_depth.png', camera_depth_data)

        elif param == 'mynt':
            depth_data = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\mynt\20200520\result\depth_matrix.npy')
            camera_data = np.genfromtxt(r'D:\工作\测试项目\深度图_全站仪\camera_data\202005220\mynt_depth_20200520\mynt_depth\mynt_depth_0.csv',delimiter=',')
            factor = 1000

            # qzy深度图
            qzy_depth_data = depth_data
            qzy_depth_data_min = 0
            qzy_depth_data_max = qzy_depth_data.max()
            qzy_depth_data_invalid = np.where(qzy_depth_data < 0)
            qzy_depth_data = (qzy_depth_data - qzy_depth_data_min) / (qzy_depth_data_max - qzy_depth_data_min) * 255
            qzy_depth_data[qzy_depth_data_invalid] = 0
            qzy_depth_data = qzy_depth_data.astype(np.uint8).T
            cv2.imshow('qzy_mynt_depth', qzy_depth_data)
            # cv2.imwrite('qzy_mynt_depth.png', qzy_depth_data)

            # mynt 深度图
            camera_depth_data = camera_data[:,0:1280]
            camera_depth_data_min = 0
            # depth_data_invalid = np.where(np.isinf(depth_data))
            # depth_data[depth_data_invalid] = 0
            camera_depth_data_max = camera_depth_data.max()
            camera_depth_data = (camera_depth_data - camera_depth_data_min) / (camera_depth_data_max - camera_depth_data_min) * 255
            camera_depth_data = camera_depth_data.astype(np.uint8)
            cv2.imshow('mynt', camera_depth_data)
            # cv2.imwrite('mynt_depth.png', camera_depth_data)

        else:
            print('输入的相机名称有误，无法通过鼠标获取深度值!!!')
            return
        # depth_data = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\leadsense\20200520\result\depth_matrix.npy')
        qzy_depth = depth_data[x][y]
        camera_depth = camera_data[y][x] / factor
        print("{0}: u:{1} v:{2} qzy_depth:{3} camera_depth:{4}".format(param, x, y, qzy_depth, camera_depth))


def get_depth_mouse(name):
    """
    通过鼠标获取图像对应的深度值
    :return:
    """

    if name == 'zed':
        img = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\zed\20200514\result\rgb_image.npy')
    elif name == 'leadsense':
        img = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\leadsense\20200520\result\rgb_image.npy')
    elif name == 'mynt':
        img = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\mynt\20200520\result\rgb_image.npy')
    else:
        print('输入的相机名称有误，无法通过鼠标获取深度值!!!')
        return

    cv2.namedWindow("image_mouse")
    cv2.setMouseCallback("image_mouse", on_EVENT_LBUTTONDOWN, param=name)
    cv2.imshow("image_mouse", img)
    cv2.waitKey(0)


def get_cheesboard_pose(A, path, num):
    """
    从获取的cheesboard2camera_pose中选取四个角点
    :param
    num:需要计算的照片的数目，对应的是4*num个角点
    :return: 合并在一起的角点
    """

    result_pose = -1
    for i in range(0, num):
        img_path = path + str(i) + ".png"
        img = cv_imread(img_path)
        result = get_cheesboard_3d_pos(img, A, is_colorful=True, opti_method="cv2_solvePnP")
        if isinstance(result, str):
            print(result)
            return
        else:
            cheesboard2camera_pose = result[0]
            # print(cheesboard2camera_pose)
            # print(result[1])
            if isinstance(result_pose, int):
                result_pose = cheesboard2camera_pose
            else:
                result_pose = np.concatenate((result_pose, cheesboard2camera_pose), axis=1)
    return result_pose


def insert_point(corner_points, xnum, ynum):
    result_matrix = np.zeros((3, xnum, ynum))
    result_matrix[:, 0, 0] = corner_points[:, 0]
    result_matrix[:, xnum - 1, 0] = corner_points[:, 1]
    result_matrix[:, 0, ynum - 1] = corner_points[:, 2]
    result_matrix[:, xnum - 1, ynum - 1] = corner_points[:, 3]
    delta_u = (corner_points[:, 1] - corner_points[:, 0]) / (xnum - 1)
    delta_d = (corner_points[:, 3] - corner_points[:, 2]) / (xnum - 1)
    for i in range(xnum - 2):
        result_matrix[:, i + 1, 0] = result_matrix[:, 0, 0] + delta_u * (i + 1)
        result_matrix[:, i + 1, ynum - 1] = result_matrix[:, 0, ynum - 1] + delta_d * (i + 1)
    for i in range(xnum):
        delta_d = (result_matrix[:, i, ynum - 1] - result_matrix[:, i, 0]) / (ynum - 1)
        for j in range(ynum - 2):
            result_matrix[:, i, j + 1] = result_matrix[:, i, 0] + delta_d * (j + 1)
    output_matrix = -1
    for i in range(ynum):
        if isinstance(output_matrix, int):
            output_matrix = result_matrix[:, :, i]
        else:
            output_matrix = np.concatenate((output_matrix, result_matrix[:, :, i]), axis=1)
    return output_matrix


def cv_imread(file_path):
    cv_img = cv2.imdecode(np.fromfile(file_path, dtype=np.uint8), -1)
    return cv_img


def estimate(qzy_chessborad_matrix, A, path, qzy2camera_RT, rt, num):
    image_list = []
    camera_pose = -1
    criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)
    # A = get_camera_internal_reference('zed')

    for i in range(0, num):

        img_path = path + str(i) + ".png"
        img = cv_imread(img_path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, (11, 8))
        corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
        if isinstance(camera_pose, int):
            camera_pose = corners2[:, 0, :]
            image_list.append(corners2[:, 0, 0:2])
        else:
            camera_pose = np.concatenate((camera_pose, corners2[:, 0, :]), axis=0)
            image_list.append(corners2[:, 0, 0:2])

    # 根据计算的初值，修改
    # estimate_matrix = np.array([1.5979, 0, -1.761, 1.587, -0.1894, 5.77, 683, 679, 683, 369])
    estimate_matrix = np.array(
        [rt[0], rt[1], rt[2], qzy2camera_RT[0][3], qzy2camera_RT[1][3], qzy2camera_RT[2][3], A[0][0], A[1][1], A[0][2],
         A[1][2]])

    corners_homo = np.concatenate((camera_pose, np.ones((camera_pose.shape[0], 1))), axis=1).T
    qzy_homo = np.concatenate((qzy_chessborad_matrix, np.ones((1, qzy_chessborad_matrix.shape[1]))))
    rt = minimize(distance_chessboard2camera_rtA, estimate_matrix,
                  args=(qzy_homo, corners_homo, False))
    rt = rt.x
    temp_Rx = np.array(
        [[1, 0, 0, 0], [0, math.cos(rt[0]), -math.sin(rt[0]), 0], [0, math.sin(rt[0]), math.cos(rt[0]), 0],
         [0, 0, 0, 1]])
    temp_Ry = np.array(
        [[math.cos(rt[1]), 0, math.sin(rt[1]), 0], [0, 1, 0, 0], [-math.sin(rt[1]), 0, math.cos(rt[1]), 0],
         [0, 0, 0, 1]])
    temp_Rz = np.array(
        [[math.cos(rt[2]), -math.sin(rt[2]), 0, 0], [math.sin(rt[2]), math.cos(rt[2]), 0, 0], [0, 0, 1, 0],
         [0, 0, 0, 1]])
    estimate_RT = np.dot(np.dot(temp_Rx, temp_Ry), temp_Rz)
    estimate_RT[:, 3] = np.array([rt[3], rt[4], rt[5], 1]).reshape(1, 4)

    estimate_A = np.array([[rt[6], 0, rt[7], 0],
                           [0, rt[8], rt[9], 0],
                           [0, 0, 1, 0]])

    print("————————————————————————————————————优化前的结果————————————————————————————————")
    print('优化前的全站仪到相机的旋转平移矩阵： ')
    print(qzy2camera_RT)
    print('\t\t')
    print('优化前的相机内参矩阵： ')
    print(A)
    print("————————————————————————————————————优化前的结果————————————————————————————————")

    print("————————————————————————————————————优化后的结果————————————————————————————————")
    print('优化后的全站仪到相机的旋转平移矩阵： ')
    print(estimate_RT)
    print('\t\t')
    # print(rt)

    print('优化后的相机内参矩阵： ')
    print(estimate_A)
    print("————————————————————————————————————优化后的结果————————————————————————————————")

    return estimate_RT, estimate_A


def get_result(name):
    # 是否使用优化后的qzy2camera_RT、相机内参；‘1‘ -> 使用； ’0‘ -> 不使用
    estimate_yes = 1
    # 相机标定的位置点数
    num = 4
    # 选用的相机名称
    print('开始相机{}的深度测评!!!'.format(name))

    # 加载全站仪点云数据
    # qzy_points_pose = np.genfromtxt(r'D:\工作\测试项目\深度图_全站仪\DepthTest\leica\leica_point_20200514.csv', delimiter=",")
    if os.path.exists(r'D:\工作\测试项目\深度图_全站仪\DepthTest\leica\leica_point_20200520.npy'):
        qzy_points_pose = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\leica\leica_point_20200520.npy')
        print('leica_point_20200520.npy 文件已经存在')
    else:
        qzy_points_pose = np.genfromtxt(r'D:\工作\测试项目\深度图_全站仪\DepthTest\leica\leica_point_20200520.csv',
                                        delimiter=",")
        np.save(r'D:\工作\测试项目\深度图_全站仪\DepthTest\leica\leica_point_20200520', qzy_points_pose)

    if name == 'zed':
        # zed
        qzy_pose = np.genfromtxt(r'D:\工作\测试项目\深度图_全站仪\DepthTest\zed\20200514\qzy\16_points.csv', delimiter=',')
        camera_depth_path = r'D:\工作\测试项目\深度图_全站仪\DepthTest\zed\20200514\camera\zed_depth.png'
        img_path = r'D:\工作\测试项目\深度图_全站仪\DepthTest\zed\20200514\camera\zed_depth_'
        A = get_camera_internal_reference('zed')
    elif name == 'leadsense':
        # leadsense
        qzy_pose = np.genfromtxt(r'D:\工作\测试项目\深度图_全站仪\DepthTest\leadsense\20200520\qzy\20200520_leadsense.csv',
                                 delimiter=',')
        camera_depth_path = r'D:\工作\测试项目\深度图_全站仪\DepthTest\leadsense\20200520\camera\leadSense_depth.png'
        img_path = r'D:\工作\测试项目\深度图_全站仪\DepthTest\leadsense\20200520\camera\leadSense_depth_'
        A = get_camera_internal_reference('leadsense')
    elif name == 'mynt':
        # mynt
        qzy_pose = np.genfromtxt(r'D:\工作\测试项目\深度图_全站仪\DepthTest\mynt\20200520\qzy\20200520_mynt.csv', delimiter=',')
        camera_depth_path = r'D:\工作\测试项目\深度图_全站仪\DepthTest\mynt\20200520\camera\mynt_depth.png'
        img_path = r'D:\工作\测试项目\深度图_全站仪\DepthTest\mynt\20200520\camera\mynt_depth_'
        A = get_camera_internal_reference('mynt')
    else:
        print('输入的相机名字有误！！！ 数据加载失败！！！')

    cheesboard2camera_pose = get_cheesboard_pose(A, img_path, num)

    # 全站仪到相机的旋转平移矩阵
    qzy_pose = qzy_pose.T
    qzy_chessborad_matrix = -1
    qzy_list = []
    image_list = []
    for i in range(0, num):
        # if i==3:
        #     continue
        if isinstance(qzy_chessborad_matrix, int):
            qzy_chessborad_matrix = insert_point(qzy_pose[:, i * 4:4 * (i + 1)], 11, 8)
            qzy_list.append(insert_point(qzy_pose[:, i * 4:4 * (i + 1)], 11, 8)[0:3, :].T.astype(np.float32))
        else:
            qzy_chessborad_matrix = np.concatenate(
                (qzy_chessborad_matrix, insert_point(qzy_pose[:, i * 4:4 * (i + 1)], 11, 8)), axis=1)
            qzy_list.append(insert_point(qzy_pose[:, i * 4:4 * (i + 1)], 11, 8)[0:3, :].T.astype(np.float32))

    qzy2camera_RT, rt = calibration_from_qzy2camera(qzy_chessborad_matrix, cheesboard2camera_pose,
                                                    opti_method="minimize")

    # 优化qzy2camera_RT, A
    estimate_RT, estimate_A = estimate(qzy_chessborad_matrix, A, img_path, qzy2camera_RT, rt, num)

    # 获取全站仪的坐标
    qzy_points_pose = qzy_points_pose.T
    qzy_points_pose = np.concatenate(
        (qzy_points_pose[:3], np.ones((1, qzy_points_pose.shape[1])), qzy_points_pose[3:]))
    qzy_xyz = qzy_points_pose[:4]

    # 是否使用优化后的结果
    if estimate_yes:
        print('使用优化后qzy2camera_RT 和 相机内参！！！')
        qzy2camera_RT = estimate_RT
        A = estimate_A
    else:
        print('未使用优化后的参数！！！')

    # 全站仪坐标转到相机像平面
    qzy_to_camera_points = np.dot(qzy2camera_RT, qzy_xyz)
    qzy_to_camera_points = np.concatenate((qzy_to_camera_points, qzy_points_pose[4:]))

    # qzy投影到相机的像平面
    depth_matrix, RGB_matrix = project_pointcloud_to_image_four_dimension(qzy_to_camera_points, A, (1280, 720))
    RGB_matrix = (RGB_matrix.transpose((2, 1, 0)) / 65536 * 255).astype(np.uint8)
    qzy_image = cv2.cvtColor(RGB_matrix, cv2.COLOR_RGB2BGR)

    diff_copy = np.zeros((720, 1280, 3))

    camera_depth_path = cv_imread(camera_depth_path)
    if camera_depth_path.shape[-1] == 4:
        camera_depth_path = cv2.cvtColor(camera_depth_path, cv2.COLOR_BGRA2BGR)
    diff_copy = (qzy_image + camera_depth_path)
    diff_copy = diff_copy.astype(np.uint8)

    if name == 'zed':

        # zed 数据存储
        np.save(r'D:\工作\测试项目\深度图_全站仪\DepthTest\zed\20200514\result\rgb_image', qzy_image)
        cv2.imwrite("rgb_image_zed.png",qzy_image,[int( cv2.IMWRITE_JPEG_QUALITY), 100])
        np.save(r'D:\工作\测试项目\深度图_全站仪\DepthTest\zed\20200514\result\depth_matrix', depth_matrix)
        np.save(r'D:\工作\测试项目\深度图_全站仪\DepthTest\zed\20200514\result\diff_img', diff_copy)
    elif name == 'leadsense':
        # leadsenes 数据存储
        np.save(r'D:\工作\测试项目\深度图_全站仪\DepthTest\leadsense\20200520\result\rgb_image', qzy_image)
        np.save(r'D:\工作\测试项目\深度图_全站仪\DepthTest\leadsense\20200520\result\depth_matrix', depth_matrix)
        np.save(r'D:\工作\测试项目\深度图_全站仪\DepthTest\leadsense\20200520\result\diff_img', diff_copy)
    elif name == 'mynt':
        # mynt 数据存储
        np.save(r'D:\工作\测试项目\深度图_全站仪\DepthTest\mynt\20200520\result\rgb_image', qzy_image)
        np.save(r'D:\工作\测试项目\深度图_全站仪\DepthTest\mynt\20200520\result\depth_matrix', depth_matrix)
        np.save(r'D:\工作\测试项目\深度图_全站仪\DepthTest\mynt\20200520\result\diff_img', diff_copy)

        # plt.imsave(r'D:\工作\测试项目\深度图_全站仪\DepthTest\mynt\20200520\result\rgb_image.png', qzy_image)
        # plt.imsave(r'D:\工作\测试项目\深度图_全站仪\DepthTest\mynt\20200520\result\depth_matrix.png', depth_matrix)
        # plt.imsave(r'D:\工作\测试项目\深度图_全站仪\DepthTest\mynt\20200520\result\diff_img.png', diff_copy)

    else:
        print('输入的相机名字有误！！！ 数据保存失败！！！')

    cv2.imshow('qzy2camera_img', qzy_image)
    cv2.imshow("test", diff_copy)
    cv2.waitKey(0)


def plot(camera_name):

    if camera_name == 'zed':
        valid_ratio = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\zed\20200514\result\valid_ratio.npy')
        evaluate_ratio = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\zed\20200514\result\evaluate_ratio.npy')
        mean_error = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\zed\20200514\result\mean_error_with_edge.npy')
        evaluate_ratio_without_edge = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\zed\20200514\result\evaluate_ratio_without_edge.npy')
        mean_error_without_edge = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\zed\20200514\result\mean_error_without_edge.npy')

        diff_error = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\zed\20200514\result\diff_error.npy')
        diff_error_without_edge = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\zed\20200514\result\diff_error_without_edge.npy')

        # plot zed_depth_color
        camera_depth_color = np.genfromtxt(
            r'D:\工作\测试项目\深度图_全站仪\camera_data\20200515\zed_depth\zed_depth_0.csv', delimiter=',')
        camera_depth_color = camera_depth_color[:, :1280]
        plt.figure()
        plt.imshow(camera_depth_color)
        plt.title('camera_depth_color')
        plt.colorbar()

        # plot qzy_depth_color
        qzy_depth_color = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\zed\20200514\result\depth_matrix.npy').T
        qzy_depth_color[np.where(qzy_depth_color < 0)] = np.nan
        plt.figure()
        plt.imshow(qzy_depth_color)
        plt.title('qzy_depth_color')
        plt.colorbar()

    elif camera_name == 'leadsense':
        valid_ratio = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\leadsense\20200520\result\valid_ratio.npy')
        evaluate_ratio = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\leadsense\20200520\result\evaluate_ratio.npy')
        mean_error = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\leadsense\20200520\result\mean_error_with_edge.npy')
        evaluate_ratio_without_edge = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\leadsense\20200520\result\evaluate_ratio_without_edge.npy')
        mean_error_without_edge = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\leadsense\20200520\result\mean_error_without_edge.npy')

        diff_error = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\leadsense\20200520\result\diff_error.npy')
        diff_error_without_edge = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\leadsense\20200520\result\diff_error_without_edge.npy')

        # plot leadsense_depth_color
        camera_depth_color = np.genfromtxt(
            r'D:\工作\测试项目\深度图_全站仪\camera_data\202005220\leadsense_depth_20200520\leadsense_depth\leadSense_depth_0.csv', delimiter=',')
        camera_depth_color = camera_depth_color[:, :1280] / 1000
        plt.figure()
        plt.imshow(camera_depth_color)
        plt.title('camera_depth_color')
        plt.colorbar()

        # plot qzy_depth_color
        qzy_depth_color = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\leadsense\20200520\result\depth_matrix.npy').T
        qzy_depth_color[np.where(qzy_depth_color < 0)] = np.nan
        plt.figure()
        plt.imshow(qzy_depth_color)
        plt.title('qzy_depth_color')
        plt.colorbar()

    elif camera_name == 'mynt':
        valid_ratio = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\mynt\20200520\result\valid_ratio.npy')
        evaluate_ratio = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\mynt\20200520\result\evaluate_ratio.npy')
        mean_error = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\mynt\20200520\result\mean_error_with_edge.npy')
        evaluate_ratio_without_edge = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\mynt\20200520\result\evaluate_ratio_without_edge.npy')
        mean_error_without_edge = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\mynt\20200520\result\mean_error_without_edge.npy')

        diff_error = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\mynt\20200520\result\diff_error.npy')
        diff_error_without_edge = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\mynt\20200520\result\diff_error_without_edge.npy')

        # plot mynt_depth_color
        camera_depth_color = np.genfromtxt(
            r'D:\工作\测试项目\深度图_全站仪\camera_data\202005220\mynt_depth_20200520\mynt_depth\mynt_depth_0.csv', delimiter=',')
        camera_depth_color = camera_depth_color[:, :1280] / 1000
        camera_depth_color[np.where(camera_depth_color <= 0)] = np.nan
        plt.figure()
        plt.imshow(camera_depth_color)
        plt.title('camera_depth_color')
        plt.colorbar()

        # plot qzy_depth_color
        qzy_depth_color = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\mynt\20200520\result\depth_matrix.npy').T
        qzy_depth_color[np.where(qzy_depth_color < 0)] = np.nan
        plt.figure()
        plt.imshow(qzy_depth_color)
        plt.title('qzy_depth_color')
        plt.colorbar()


    print('输出相机{} 的测试结果：'.format(camera_name))
    print(' valid_ratio:\n {0} \n\n evaluate_ratio:\n {1} \n\n mean_error:\n {2} \n\n evaluate_ratio_without_edge:\n {3} \n\n mean_error_without_edge:\n {4}'.format
          (valid_ratio, evaluate_ratio, mean_error, evaluate_ratio_without_edge, mean_error_without_edge))
    print('valid_ratio:{0} \t evaluate_ratio:{1} \t mean_error:{2} \t evaluate_ratio_without_edge:{3} \t mean_error_without_edge:{4}'.format(valid_ratio.mean(), evaluate_ratio.mean(), mean_error.mean(), evaluate_ratio_without_edge.mean(), mean_error_without_edge.mean()))


    diff_error_0 = diff_error[0]
    diff_error_0[np.where(diff_error_0 <= 0)] = np.nan
    diff_error_0[np.where(np.isnan(qzy_depth_color))] = np.nan
    diff_error_0[np.where(np.isnan(camera_depth_color))] = np.nan

    plt.figure()
    plt.imshow(diff_error_0)
    plt.title('diff_error')
    plt.colorbar()

    #
    diff_error_without_edge_0 = diff_error_without_edge[0]
    diff_error_without_edge[np.where(diff_error_without_edge <= 0)] = np.nan
    diff_error_without_edge_0[np.where(np.isnan(qzy_depth_color))] = np.nan
    diff_error_without_edge_0[np.where(np.isnan(camera_depth_color))] = np.nan

    plt.figure()
    plt.imshow(diff_error_without_edge_0)
    plt.title('diff_error_without_edge_0')
    plt.colorbar()


    plt.show()

    # cv2.waitKey(0)


if __name__ == '__main__':
    # 测试的相机 ['zed'、'leadsense'、'mynt']
    camera_name = 'zed'
    # camera_name = 'leadsense'
    # camera_name = 'mynt'

    # 获取测试结果
    get_result(camera_name)

    # 深度评估
    Evaluate_depth.evaluate_depth(camera_name)

    # 通过鼠标获取全站仪转到像平面的深度值
    # get_depth_mouse(camera_name)

    # 画图
    # plot(camera_name)






