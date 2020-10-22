import cv2
import numpy as np


def cv_imread(file_path):
    cv_img = cv2.imdecode(np.fromfile(file_path, dtype=np.uint8), -1)
    return cv_img

# zed
# img_path = r"D:\工作\测试项目\深度图_全站仪\DepthTest\zed\20200514\result\test.png"
# imgc_path = r"D:\工作\测试项目\深度图_全站仪\DepthTest\zed\20200514\camera\zed_depth.png"

# leadsense
# img_path = r"D:\工作\测试项目\深度图_全站仪\DepthTest\zed\20200514\result\test.png"
# imgc_path = r"D:\工作\测试项目\深度图_全站仪\DepthTest\zed\20200514\camera\zed_depth.png"

# mynt
# img_path = r"D:\工作\测试项目\深度图_全站仪\DepthTest\zed\20200514\result\test.png"
# imgc_path = r"D:\工作\测试项目\深度图_全站仪\DepthTest\zed\20200514\camera\zed_depth.png"

img = cv_imread(img_path)
imgc = cv_imread(imgc_path)
imgc = cv2.cvtColor(imgc, cv2.COLOR_BGRA2BGR)
diff_copy = np.zeros((720, 1280, 3))

cv2.imshow('test1', imgc)
diff_copy = (imgc + img)
diff_copy = diff_copy.astype(np.uint8)
cv2.imshow("test", diff_copy)
cv2.waitKey(0)
