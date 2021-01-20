import numpy as np
import cv2


# 评估照片的数目
num = 10
show_true = 0

valid_ratio = []
evaluate_ratio = []
mean_error = []
evaluate_ratio_without_edge = []
mean_error_without_edge = []

diff_error = []
diff_error_without_edge = []


def cv_imread(file_path):
    cv_img = cv2.imdecode(np.fromfile(file_path, dtype=np.uint8), -1)
    return cv_img


def evaluate_depth(camera_name, Camera_depth_path, depth_tail):
    print('开始相机深度图评估！！！')

    for i in range(num):

        camera_path = Camera_depth_path + depth_tail + str(i) + '.csv'
        camera_depth_matrix = np.genfromtxt(camera_path, delimiter=',')
        gtdepth = np.load(r'result\depth_matrix.npy').T
        camera_image = cv_imread(Camera_depth_path + depth_tail + '0.png')
        qzy_image = np.load(r'result\rgb_image.npy')

        if camera_name == 'zed':
            factor = 1

        elif camera_name == 'leadsense':
            factor = 1000

        elif camera_name == 'mynt':
            factor = 1000

        elif camera_name == 'D435':
            factor = 1

        elif camera_name == 'D435i':
            factor = 1

        else:
            print('【error】请确认输入的相机名称是否正确！！！')

        camera_depth_matrix = camera_depth_matrix[0:720, :1280] / factor
        total_pixel = 720 * 1280
        if camera_name == 'zed':
            nan_numbers = len(camera_depth_matrix[np.where(np.isnan(camera_depth_matrix))])
            inf_numbers = 0
            below_zero_numbers = 0
        elif camera_name == 'leadsense':
            nan_numbers = 0
            inf_numbers = len(camera_depth_matrix[np.where(np.isinf(camera_depth_matrix))])
            below_zero_numbers = 0
        elif camera_name == 'mynt':
            nan_numbers = 0
            inf_numbers = 0
            below_zero_numbers = len(camera_depth_matrix[np.where(camera_depth_matrix <= 0)])
        elif camera_name == 'D435':
            nan_numbers = 0
            inf_numbers = 0
            below_zero_numbers = len(camera_depth_matrix[np.where(camera_depth_matrix <= 0)])
        elif camera_name == 'D435i':
            nan_numbers = 0
            inf_numbers = 0
            below_zero_numbers = len(camera_depth_matrix[np.where(camera_depth_matrix <= 0)])
        else:
            print('Error!!!')

        invalid_numbers = nan_numbers + inf_numbers + below_zero_numbers
        print('相机：' +  camera_name + '的深度比测测试结果：')
        print("invalid numbers:{0},where nan:{1},inf:{2},<=0:{3}".format(invalid_numbers, nan_numbers, inf_numbers,
                                                                         below_zero_numbers))
        valid_ratio_temp = 1 - invalid_numbers / total_pixel
        valid_ratio.append(valid_ratio_temp)
        # print("valid_ratio:", valid_ratio)

        valid_postion = np.where(
            (np.isnan(camera_depth_matrix) == False) & (gtdepth != -1) & (np.isinf(camera_depth_matrix) == False) & (
                    camera_depth_matrix > 0))
        valid_numbers = len(valid_postion[0])
        evaluate_ratio_temp = valid_numbers / total_pixel
        evaluate_ratio.append(evaluate_ratio_temp)
        # print("evaluate_ratio", evaluate_ratio)
        diff = np.zeros((720, 1280))
        for i in range(valid_numbers):
            u = valid_postion[0][i]
            v = valid_postion[1][i]
            diff[u, v] = abs(gtdepth[u, v] - camera_depth_matrix[u, v])
        diff_error.append(diff)
        mean_error_temp = diff.sum() / valid_numbers
        mean_error.append(mean_error_temp)
        # print("mean_error_with_edge:", mean_error)
        show_diff_gray = (diff / (diff.max() - diff.min()) * (255)).astype(np.uint8)

        if show_true:
            cv2.imshow("diff", show_diff_gray)
            cv2.waitKey()

        if camera_image.shape[-1] == 4:
            camera_depth_path = cv2.cvtColor(camera_image, cv2.COLOR_BGRA2BGR)
        show_image = np.zeros((720, 1280, 3), dtype=np.uint8)
        blur = cv2.GaussianBlur(camera_image, (3, 3), 0)  # 用高斯滤波处理原图像降噪
        canny = cv2.Canny(blur, 50, 150)  # 50是最小阈值,150是最大阈值
        show_image[:, :, 0] = canny
        if show_true:
            cv2.imshow('camera_canny', canny)
            cv2.waitKey()

        qzy_image = cv2.cvtColor(qzy_image, cv2.COLOR_RGB2BGR)
        blur = cv2.GaussianBlur(qzy_image, (3, 3), 0)  # 用高斯滤波处理原图像降噪
        canny = cv2.Canny(blur, 50, 150)  # 50是最小阈值,150是最大阈值
        show_image[:, :, 2] = canny
        if show_true:
            cv2.imshow('qzy_canny', canny)
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
                no_edge_diff[u, v] = abs(gtdepth[u, v] - camera_depth_matrix[u, v])
        diff_error_without_edge.append(no_edge_diff)
        evaluate_ratio_without_edge_temp = (valid_numbers - edge_number) / total_pixel
        evaluate_ratio_without_edge.append(evaluate_ratio_without_edge_temp)
        # print("evaluate_ratio_without_edge", evaluate_ratio_without_edge)
        mean_error_without_edge_temp = no_edge_diff.sum() / (valid_numbers - edge_number)
        mean_error_without_edge.append(mean_error_without_edge_temp)
        # print("mean_error_without_edge:", mean_error_without_edge)
        show_diff_gray = (no_edge_diff / (no_edge_diff.max() - no_edge_diff.min()) * (255)).astype(np.uint8)
        if show_true:
            cv2.imshow("diff_without_edge", show_diff_gray)
            cv2.waitKey()
    # 打印结果

    np.save(r'result\valid_ratio', valid_ratio)
    np.save(r'result\evaluate_ratio', evaluate_ratio)
    np.save(r'result\mean_error_with_edge', mean_error)
    np.save(r'result\evaluate_ratio_without_edge', evaluate_ratio_without_edge)
    np.save(r'result\mean_error_without_edge', mean_error_without_edge)

    np.save(r'result\diff_error', diff_error)
    np.save(r'result\diff_error_without_edge', diff_error_without_edge)


    print("valid_ratio:", valid_ratio)
    print("evaluate_ratio", evaluate_ratio)
    print("mean_error_with_edge:", mean_error)
    print("evaluate_ratio_without_edge", evaluate_ratio_without_edge)
    print("mean_error_without_edge:", mean_error_without_edge)
