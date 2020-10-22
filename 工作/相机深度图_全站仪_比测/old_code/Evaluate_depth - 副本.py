import numpy as np
import cv2


def cv_imread(file_path):
    cv_img = cv2.imdecode(np.fromfile(file_path, dtype=np.uint8), -1)
    return cv_img


def evaluate_depth(camera_name):
    print('开始相机深度图评估！！！')

    if camera_name == 'zed':
        zed_depth_matrix = np.genfromtxt(r'D:\工作\测试项目\深度图_全站仪\camera_data\20200515\zed_depth\zed_depth_0.csv',
                                         delimiter=',')
        gtdepth = np.load(r"D:\工作\测试项目\深度图_全站仪\DepthTest\zed\20200514\result\depth_matrix.npy").T
        camera_image = cv_imread(r"D:\工作\测试项目\深度图_全站仪\DepthTest\zed\20200514\camera\zed_depth.png")
        qzy_image = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\zed\20200514\result\rgb_image.npy')
        factor = 1

    elif camera_name == 'leadsense':
        gtdepth = np.load(r"D:\工作\测试项目\深度图_全站仪\DepthTest\leadsense\20200520\result\depth_matrix.npy").T
        zed_depth_matrix = np.genfromtxt(
            r'D:\工作\测试项目\深度图_全站仪\camera_data\202005220\leadsense_depth_20200520\leadsense_depth\leadSense_depth_1.csv',
            delimiter=',')
        camera_image = cv_imread(r"D:\工作\测试项目\深度图_全站仪\DepthTest\leadsense\20200520\camera\leadSense_depth.png")
        qzy_image = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\leadsense\20200520\result\rgb_image.npy')
        factor = 1000

    elif camera_name == 'mynt':
        gtdepth = np.load(r"D:\工作\测试项目\深度图_全站仪\DepthTest\mynt\20200520\result\depth_matrix.npy").T
        zed_depth_matrix = np.genfromtxt(
            r'D:\工作\测试项目\深度图_全站仪\camera_data\202005220\mynt_depth_20200520\mynt_depth\mynt_depth_0.csv', delimiter=',')
        camera_image = cv_imread(r"D:\工作\测试项目\深度图_全站仪\DepthTest\mynt\20200520\camera\mynt_depth.png")
        qzy_image = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\mynt\20200520\result\rgb_image.npy')
        factor = 1000

    else:
        print('【error】请确认输入的相机名称是否正确！！！')

    zed_depth_matrix = zed_depth_matrix[0:720, :1280] / factor
    total_pixel = 720 * 1280
    nan_numbers = len(zed_depth_matrix[np.where(np.isnan(zed_depth_matrix))])
    inf_numbers = len(zed_depth_matrix[np.where(np.isinf(zed_depth_matrix))])
    below_zero_numbers = len(zed_depth_matrix[np.where(zed_depth_matrix <= 0)])
    invalid_numbers = nan_numbers + inf_numbers + below_zero_numbers
    print('相机：' +  camera_name + '的深度比测测试结果：')
    print("invalid numbers:{0},where nan:{1},inf:{2},<=0:{3}".format(invalid_numbers, nan_numbers, inf_numbers,
                                                                     below_zero_numbers))
    validratio = 1 - invalid_numbers / total_pixel
    print("valid_ratio:", validratio)
    # gtdepth = np.load(r"D:\工作\测试项目\深度图_全站仪\DepthTest\zed\20200514\result\depth_matrix.npy").T
    # gtdepth = np.load(r"D:\工作\测试项目\深度图_全站仪\DepthTest\leadsense\20200520\result\depth_matrix.npy").T
    # gtdepth = np.load(r"D:\工作\测试项目\深度图_全站仪\DepthTest\mynt\20200520\result\depth_matrix.npy").T
    valid_postion = np.where(
        (np.isnan(zed_depth_matrix) == False) & (gtdepth != -1) & (np.isinf(zed_depth_matrix) == False) & (
                zed_depth_matrix > 0))
    valid_numbers = len(valid_postion[0])
    evaluate_ratio = valid_numbers / total_pixel
    print("evaluate_ratio", evaluate_ratio)
    diff = np.zeros((720, 1280))
    for i in range(valid_numbers):
        u = valid_postion[0][i]
        v = valid_postion[1][i]
        diff[u, v] = abs(gtdepth[u, v] - zed_depth_matrix[u, v])
    mean_error = diff.sum() / valid_numbers
    print("mean_error_with_edge:", mean_error)
    show_diff_gray = (diff / (diff.max() - diff.min()) * (255)).astype(np.uint8)
    # show_diff=(diff/(diff.max()-diff.min())*(256**3-1))
    # colorful_diff=np.zeros((720,1280,3))
    # colorful_diff[:,:,2]=(show_diff/(256**2)).astype(np.uint8)
    # colorful_diff[:,:,1]=(show_diff%(256**2)/256).astype(np.uint8)
    # colorful_diff[:,:,0]=(show_diff%256).astype(np.uint8)
    # colorful_diff=cv2.cvtColor(show_diff_gray,cv2.COLOR_GRAY2BGR)
    cv2.imshow("diff", show_diff_gray)
    cv2.waitKey()
    # cv2.imshow("colorful_diff",colorful_diff)
    # cv2.waitKey()
    # camera_image = cv_imread(r"D:\工作\测试项目\深度图_全站仪\DepthTest\zed\20200514\camera\zed_depth.png")
    # camera_image = cv_imread(r"D:\工作\测试项目\深度图_全站仪\DepthTest\leadsense\20200520\camera\leadSense_depth.png")
    # camera_image = cv_imread(r"D:\工作\测试项目\深度图_全站仪\DepthTest\mynt\20200520\camera\mynt_depth.png")
    if camera_image.shape[-1] == 4:
        camera_depth_path = cv2.cvtColor(camera_image, cv2.COLOR_BGRA2BGR)
    show_image = np.zeros((720, 1280, 3), dtype=np.uint8)
    blur = cv2.GaussianBlur(camera_image, (3, 3), 0)  # 用高斯滤波处理原图像降噪
    canny = cv2.Canny(blur, 50, 150)  # 50是最小阈值,150是最大阈值
    show_image[:, :, 0] = canny
    cv2.imshow('canny', canny)
    cv2.waitKey()
    # qzy_image = cv2.imread(r"D:\python_code\calibration\DepthTest\test.png")
    # qzy_image = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\zed\20200514\result\rgb_image.npy')
    # qzy_image = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\leadsense\20200520\result\rgb_image.npy')
    # qzy_image = np.load(r'D:\工作\测试项目\深度图_全站仪\DepthTest\mynt\20200520\result\rgb_image.npy')
    qzy_image = cv2.cvtColor(qzy_image, cv2.COLOR_RGB2BGR)
    blur = cv2.GaussianBlur(qzy_image, (3, 3), 0)  # 用高斯滤波处理原图像降噪
    canny = cv2.Canny(blur, 50, 150)  # 50是最小阈值,150是最大阈值
    show_image[:, :, 2] = canny
    cv2.imshow('canny', canny)
    cv2.waitKey()
    cv2.imshow("campare_edge", show_image)
    cv2.waitKey()
    no_edge_diff = np.zeros((720, 1280))
    edge_number = 0
    for i in range(valid_numbers):
        u = valid_postion[0][i]
        v = valid_postion[1][i]
        edge_flag = False
        for m in range(3):
            for n in range(3):
                tempu = min(max(u + m - 1, 0), 719)
                tempv = min(max(v + n - 1, 0), 1279)
                if canny[tempu, tempv] == 255:
                    edge_flag = True
                    edge_number += 1
                    break
            if edge_flag:
                break
        if edge_flag == False:
            no_edge_diff[u, v] = abs(gtdepth[u, v] - zed_depth_matrix[u, v])
    evaluate_ratio_without_edge = (valid_numbers - edge_number) / total_pixel
    print("evaluate_ratio_without_edge", evaluate_ratio_without_edge)
    mean_error_without_edge = no_edge_diff.sum() / (valid_numbers - edge_number)
    print("mean_error_without_edge:", mean_error_without_edge)
    show_diff_gray = (no_edge_diff / (no_edge_diff.max() - no_edge_diff.min()) * (255)).astype(np.uint8)
    cv2.imshow("diff", show_diff_gray)
    cv2.waitKey()
