import numpy as np
import math
from scipy.optimize import leastsq
import cv2


def distance_chessboard2camera_rt(rt, x, y, A):
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
    return error[0]


def get_cheesboard_3d_pos(img, internal_matrix, chessboard_xlen=0.03, chessboard_ylen=0.03, chessboard_xnum=11,
                          chessboard_ynum=8, is_colorful=True, cv2_input_type=True):
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
    :param is_colorful:
    :param cv2_input_type: is True whne the input image is read by opencv
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
    rt = leastsq(distance_chessboard2camera_rt, estimate_matrix,
                 args=(chessboard_corners_homo, corners_homo, internal_matrix))
    temp_Rx = np.array(
        [[1, 0, 0, 0], [0, math.cos(rt[0][0]), -math.sin(rt[0][0]), 0], [0, math.sin(rt[0][0]), math.cos(rt[0][0]), 0],
         [0, 0, 0, 1]])
    temp_Ry = np.array(
        [[math.cos(rt[0][1]), 0, math.sin(rt[0][1]), 0], [0, 1, 0, 0], [-math.sin(rt[0][1]), 0, math.cos(rt[0][1]), 0],
         [0, 0, 0, 1]])
    temp_Rz = np.array(
        [[math.cos(rt[0][2]), -math.sin(rt[0][2]), 0, 0], [math.sin(rt[0][2]), math.cos(rt[0][2]), 0, 0], [0, 0, 1, 0],
         [0, 0, 0, 1]])
    temp_RT = np.dot(np.dot(temp_Rx, temp_Ry), temp_Rz)
    temp_RT[:, 3] = np.array([rt[0][3], rt[0][4], rt[0][5], 1]).reshape(1, 4)
    pos = np.dot(temp_RT, chessboard_corners_homo)
    return pos, temp_RT


def distance_qzy2camera_rt(rt, x, y):
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
    return error[0]


def calibration_from_qzy2camera(qzy_pos_matrix, camera_pos_matrix):
    """
    :param qzy_pos_matrix: 3d position in qzy coordinate with the shape (3*N) or (4*N)
    :param camera_pos_matrix: 3d position in camera coordinate with the shape (3*N) or (4*N)
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
    elif qzy_pos_matrix.shape[0]!=4:
        return "【ERROR】: error with input qzy_pos_matrix shape"
    if len(camera_pos_matrix.shape)!=2:
        return "【ERROR】: error with input camera_pos_matrix shape"
    if camera_pos_matrix.shape[0] ==3:
        camera_pos_matrix=np.concatenate((camera_pos_matrix,np.ones(1,camera_pos_matrix.shape[1])))
    elif camera_pos_matrix.shape[0]!=4:
        return "【ERROR】: error with input camera_pos_matrix shape"
    if qzy_pos_matrix.shape[1]!=camera_pos_matrix[1]:
        return "【ERROR】: the shape of qzy_pos_matrix and camera_pos_matrix should be same"
    estimate_matrix = np.array([1, 1, 1, 1, 1, 1])
    rt = leastsq(distance_qzy2camera_rt, estimate_matrix, args=(qzy_pos_matrix, camera_pos_matrix))
    temp_Rx = np.array(
            [[1, 0, 0, 0], [0, math.cos(rt[0][0]), -math.sin(rt[0][0]), 0], [0, math.sin(rt[0][0]), math.cos(rt[0][0]), 0],
             [0, 0, 0, 1]])
    temp_Ry = np.array(
            [[math.cos(rt[0][1]), 0, math.sin(rt[0][1]), 0], [0, 1, 0, 0], [-math.sin(rt[0][1]), 0, math.cos(rt[0][1]), 0],
             [0, 0, 0, 1]])
    temp_Rz = np.array(
            [[math.cos(rt[0][2]), -math.sin(rt[0][2]), 0, 0], [math.sin(rt[0][2]), math.cos(rt[0][2]), 0, 0], [0, 0, 1, 0],
             [0, 0, 0, 1]])
    temp_RT = np.dot(np.dot(temp_Rx, temp_Ry),temp_Rz)
    temp_RT[:, 3] = np.array([rt[0][3], rt[0][4], rt[0][5], 1]).reshape(1, 4)
    return temp_RT


# ***usage***
image_path = r"D:\xinyu\DepthTest\50.png"
img = cv2.imread(image_path)
cv2.imshow("img", img)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
A = np.array([[677.703, 0, 640, 0],
              [0, 677.703, 360, 0],
              [0, 0, 1, 0]])
# result=get_cheesboard_3d_pos(gray,A,is_colorful=False)
result = get_cheesboard_3d_pos(img, A, is_colorful=True)
if isinstance(result, str):
    print(result)
else:
    pos = result[0]
    rt_matrix = result[1]
    print(rt_matrix)
    print(pos.shape)
    print(pos)
