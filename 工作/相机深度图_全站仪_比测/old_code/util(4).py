import numpy as np
import math
from scipy.optimize import leastsq
from scipy.optimize import minimize
import cv2


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
        if len(shape) != 3:
            return "【ERROR】: error with input image shape"
        if img.shape[-1] != 3:
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
    if opti_method not in ["leastsq", "minimize"]:
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
    else:
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
    pos = np.dot(temp_RT, chessboard_corners_homo)
    return pos, temp_RT


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
    if use_leastsq:
        return error[0]
    else:
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
        qzy_pos_matrix = np.concatenate((qzy_pos_matrix, np.ones(1, qzy_pos_matrix.shape[1])))
    elif qzy_pos_matrix.shape[0] != 4:
        return "【ERROR】: error with input qzy_pos_matrix shape"
    if len(camera_pos_matrix.shape) != 2:
        return "【ERROR】: error with input camera_pos_matrix shape"
    if camera_pos_matrix.shape[0] == 3:
        camera_pos_matrix = np.concatenate((camera_pos_matrix, np.ones(1, camera_pos_matrix.shape[1])))
    elif camera_pos_matrix.shape[0] != 4:
        return "【ERROR】: error with input camera_pos_matrix shape"
    if qzy_pos_matrix.shape[1] != camera_pos_matrix[1]:
        return "【ERROR】: the shape of qzy_pos_matrix and camera_pos_matrix should be same"
    if opti_method not in ["leastsq", "minimize"]:
        return "【ERROR】: error with optimize method now support method is leastsq and minimize from scipy.optimize"
    estimate_matrix = np.array([1, 1, 1, 1, 1, 1])
    if opti_method == "leastsq":
        rt = leastsq(distance_qzy2camera_rt, estimate_matrix, args=(qzy_pos_matrix, camera_pos_matrix, True))
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
    return temp_RT


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
        if temp_u >= 0 and temp_u <= image_size[0] and temp_v >= 0 and temp_v <= image_size[1] and temp_z>0:
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


def project_pointcloud_to_image_with_six_dimension(point_cloud_matrix, internal_matrix, image_size):
    """
    :param point_cloud_matrix: point cloud from qzy shape(6,N) or (7,N) with RGB value
    :param internal_matrix: internal matrix of camera with shape (3,3) or (3,4) or (5,6) or (6,7)
                                                                        [f 0 u 0 0 0 0
                                                                         0 f v 0 0 0 0
                                                                         0 0 1 0 0 0 0
                                                                         0 0 0 0 1 0 0
                                                                         0 0 0 0 0 1 0
                                                                         0 0 0 0 0 0 1]
    :param image_size: output images size with W*H
    :return: "str" if with some error during this function the return is the error info with the type of string
    else return "depth_matrix ,RGB_matrix" depth matrix with shape W*H and default value -1 RGB_matrix with shape 3*W*H and default value 0
    """
    """
    transform point_cloud_matrix from (4,N) to (9,N) which is [x y z 1 1 1 R G B]T then transform it to [ x/z y/z 1 x y z R G B]T
    the final project matrix 8x9 is like [f 0 u 0 0 0 0 0 0 
                                          0 f v 0 0 0 0 0 0
                                          0 0 0 1 0 0 0 0 0
                                          0 0 0 0 1 0 0 0 0
                                          0 0 0 0 0 1 0 0 0
                                          0 0 0 0 0 0 1 0 0
                                          0 0 0 0 0 0 0 1 0
                                          0 0 0 0 0 0 0 0 1]
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
    if point_cloud_matrix.shape[0] == 6:
        point_cloud_matrix = np.concatenate(
            (point_cloud_matrix[0:3], np.ones((3, point_cloud_matrix.shape[1])), point_cloud_matrix[3:6]))
    elif point_cloud_matrix.shape[0]==7:
        point_cloud_matrix = np.concatenate(
            (point_cloud_matrix[0:4], np.ones((2, point_cloud_matrix.shape[1])), point_cloud_matrix[4:7]))
    elif point_cloud_matrix.shape[0] != 9:
        return "【ERROR】error with the input point_cloud_matrix shape"
    if internal_matrix.shape[0] == 3 and internal_matrix.shape[1] == 3:
        internal_matrix = np.concatenate(
            (internal_matrix, np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]])), axis=1)
        internal_matrix = np.concatenate((internal_matrix, np.array(
            [[0, 0, 0, 0, 1, 0, 0, 0, 0], [0, 0, 0, 0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 0, 0, 1, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 0, 0, 0, 1]])))
    elif internal_matrix.shape[0] == 3 and internal_matrix.shape[1] == 4:
        internal_matrix = np.concatenate(
            (internal_matrix, np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]])), axis=1)
        internal_matrix = np.concatenate(
            (internal_matrix, np.array(
                [[0, 0, 0, 0, 1, 0, 0, 0, 0], [0, 0, 0, 0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 0, 0, 1, 0, 0],
                 [0, 0, 0, 0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 0, 0, 0, 1]])))
    elif internal_matrix.shape[0] == 5 and internal_matrix.shape[1] == 6:
        internal_matrix = np.concatenate((internal_matrix[0:2],
                                          np.array([[0, 0, 1, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]]),
                                          internal_matrix[2:5]))
        internal_matrix = np.concatenate((internal_matrix[:, 0:3], np.zeros((8, 3)), internal_matrix[:, 3:6]), axis=1)
        internal_matrix[3, 4] = 1
        internal_matrix[4, 5] = 1
    elif internal_matrix.shape[0] == 6 and internal_matrix[1] == 7:
        internal_matrix = np.concatenate((internal_matrix[0:3], np.zeros((2, 7)), internal_matrix[3:6]))
        internal_matrix = np.concatenate((internal_matrix[:, 0:4], np.zeros((8, 2)), internal_matrix[:, 4:7]), axis=1)
        internal_matrix[3, 4] = 1
        internal_matrix[4, 5] = 1
    elif internal_matrix.shape[0] != 8 and internal_matrix[1] != 9:
        return "【ERROR】: input internal matrix shape should be 8*9 with RGB"
    RGB_flag = True
    if point_cloud_matrix.shape[0] == 4:
        RGB_flag = False
    internal_matrix[2, 2] = 0
    internal_matrix[2, 3] = 1
    point_cloud_matrix[3] = point_cloud_matrix[0]
    point_cloud_matrix[4] = point_cloud_matrix[1]
    point_cloud_matrix[5] = point_cloud_matrix[2]
    point_cloud_matrix[0] /= point_cloud_matrix[2]
    point_cloud_matrix[1] /= point_cloud_matrix[2]
    point_cloud_matrix[2] /= point_cloud_matrix[2]
    uv_xyz_rgb = np.dot(internal_matrix, point_cloud_matrix)
    uv_point_info_dict = {}
    for i in range(len(uv_xyz_rgb[0])):
        temp_u = uv_xyz_rgb[0,i]
        temp_v = uv_xyz_rgb[1,i]
        if temp_u >= 0 and temp_u <= image_size[0] and temp_v >= 0 and temp_v <= image_size[1]:
            temp_u = int(temp_u)
            temp_v = int(temp_v)
            if "{0},{1}".format(temp_u, temp_v) in uv_point_info_dict.keys():
                uv_point_info_dict["{0},{1}".format(temp_u, temp_v)].append(uv_xyz_rgb[:, i])
            else:
                uv_point_info_dict["{0},{1}".format(temp_u, temp_v)] = [uv_xyz_rgb[:, i]]
    xyz_matrix = np.zeros((3, image_size[0], image_size[1]))
    if RGB_flag:
        RGB_matrix = np.zeros((3, image_size[0], image_size[1]))
    for key, value in uv_point_info_dict.items():
        point_index = key.split(',')
        temp_u = int(point_index[0])
        temp_v = int(point_index[1])
        if len(value) == 1:
            xyz_matrix[0, temp_u, temp_v] = value[0][2]
            xyz_matrix[1, temp_u, temp_v] = value[0][3]
            xyz_matrix[2, temp_u, temp_v] = value[0][4]
            if RGB_flag:
                RGB_matrix[0, temp_u, temp_v] = value[0][5]
                RGB_matrix[1, temp_u, temp_v] = value[0][6]
                RGB_matrix[2, temp_u, temp_v] = value[0][7]
        else:
            temp_array = np.array(value)
            # distance = np.sqrt((temp_array[:, 0] - temp_u) ** 2 + (temp_array[:, 1] - temp_v) ** 2)
            distance = temp_array[:, 4]
            if max(temp_array[:, 4]) - min(temp_array[:, 4]) > 0.05:
                index = np.argmin(distance)[0]
                xyz_matrix[0, temp_u, temp_v] = value[index][2]
                xyz_matrix[1, temp_u, temp_v] = value[index][3]
                xyz_matrix[2, temp_u, temp_v] = value[index][4]
                if RGB_flag:
                    RGB_matrix[0, temp_u, temp_v] = value[index][5]
                    RGB_matrix[1, temp_u, temp_v] = value[index][6]
                    RGB_matrix[2, temp_u, temp_v] = value[index][7]
            else:
                distance = np.sqrt((temp_array[:, 0] - temp_u) ** 2 + (temp_array[:, 1] - temp_v) ** 2)
                temp_distance = 1 / distance
                sum_distance = temp_distance.sum()
                weight = temp_distance / sum_distance
                weighted_value = np.dot(weight, temp_array)
                xyz_matrix[0, temp_u, temp_v] = weighted_value[2]
                xyz_matrix[1, temp_u, temp_v] = weighted_value[3]
                xyz_matrix[2, temp_u, temp_v] = weighted_value[4]
                if RGB_flag:
                    RGB_matrix[0, temp_u, temp_v] = weighted_value[5]
                    RGB_matrix[1, temp_u, temp_v] = weighted_value[6]
                    RGB_matrix[2, temp_u, temp_v] = weighted_value[7]
    if RGB_flag:
        return xyz_matrix, RGB_matrix
    else:
        return xyz_matrix, None

# ***usage***
# image_path = r"D:\python_code\calibration\left\50.png"
# image_path=r"D:\python_code\calibration\chessboard.png"
# img = cv2.imread(image_path)
# gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# A = np.array([[677.703, 0, 640, 0],
#               [0, 677.703, 360, 0],
#               [0, 0, 1, 0]])
# # result=get_cheesboard_3d_pos(gray,A,is_colorful=False)
# result = get_cheesboard_3d_pos(img, A, is_colorful=True,opti_method="minimize")
# if isinstance(result, str):
#     print(result)
# else:
#     pos = result[0]
#     rt_matrix = result[1]
#     print(rt_matrix)
#     print(pos.shape)
#     print(pos)

point = np.load("point.npy")
# point = np.insert(point, 3, np.array([[0], [0], [0], [0], [0], [0]]), axis=0)
point_mat = np.array(point)

# print(point_mat.shape)
# print(type(point_mat))
# print(point_mat)

# A = np.array([[1000, 0, 360, 0, 0, 0, 0, 0, 0],#               [0, 1000, 150, 0, 0, 0, 0, 0, 0],
#               [0, 0, 0, 1, 0, 0, 0, 0, 0],
#               [0, 0, 0, 0, 1, 0, 0, 0, 0],
#               [0, 0, 0, 0, 0, 1, 0, 0, 0],
#               [0, 0, 0, 0, 0, 0, 1, 0, 0],
#               [0, 0, 0, 0, 0, 0, 0, 1, 0],
#               [0, 0, 0, 0, 0, 0, 0, 0, 1]])
# xyz_matrix, RGB_matrix = project_pointcloud_to_image_with_six_dimension(point_mat, A, (720, 300))
# import cv2
# img=RGB_matrix.transpose((1,2,0)).astype(np.uint8)
# show_img=cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
# cv2.imshow("test",show_img)
# cv2.waitKey(0)


# pointcloud_to_image_four_dimension
A = np.array([[677.703, 0, 640, 0],
              [0, 677.703, 360, 0],
              [0, 0, 0, 1]])
point = np.genfromtxt(r"D:\工作\测试项目\深度图_全站仪\leica\leica_point.csv", delimiter=",")
np.save("point", point)
qzy_points_pose = np.load('point.npy')
RT_minimize = np.load('RT_minimize.npy')
print(RT_minimize)
qzy_points_pose = qzy_points_pose.T
print(qzy_points_pose.shape)
qzy_points_pose = np.concatenate((qzy_points_pose[:3], np.ones((1, qzy_points_pose.shape[1])), qzy_points_pose[3:]))
qzy_xyz = qzy_points_pose[:4]
qzy_to_camera = np.dot(RT_minimize, qzy_xyz)
qzy_to_camera = np.concatenate((qzy_to_camera, qzy_points_pose[4:]))
np.save('qzy_to_camera_points', qzy_to_camera)
print(qzy_to_camera.shape)

depth_matrix, RGB_matrix = project_pointcloud_to_image_four_dimension(qzy_to_camera, A, (1280, 720))
RGB_matrix=(RGB_matrix.transpose((2,1,0))/65536*255).astype(np.uint8)
show_img=cv2.cvtColor(RGB_matrix,cv2.COLOR_RGB2BGR)
cv2.imshow("test",show_img)
cv2.imwrite('test.png', show_img)
cv2.waitKey(0)


# np.save('depth_matrix', depth_matrix)
# np.save('RGB_matrix', RGB_matrix)

# depth_matrix = np.load('depth_matrix.npy')
# RGB_matrix = np.load('RGB_matrix.npy')
# print(RGB_matrix.shape)
# print(depth_matrix.shape)
# print(depth_matrix)
# print(RGB_matrix)
#
# cv2.imshow('qzy_to_camera', RGB_matrix)
# cv2.waitKey(0)

