import cv2
import numpy as np
import os
file_dir=r"D:\工作\测试项目\深度图_全站仪\20201026_camera_leica_calibration\usb_cam_20201027\left\depth"

def cv_imread(file_path):
    cv_img = cv2.imdecode(np.fromfile(file_path, dtype=np.uint8), -1)
    return cv_img

for root, dirs, files in os.walk(file_dir):
    for each_file in files:
        if "jpg" in each_file:
            img_file_path=os.path.join(root,each_file)
            img=cv_imread(img_file_path)
            # cv2.imshow("raw",img)

            #　ｍiddle
            # fx=4.00659822e+03
            # fy=4.00529212e+03
            # cx=9.35658808e+02
            # cy=4.58408128e+02
            # distcoeffs = np.array([-6.33120494e-01, 1.52092189e-01, 4.32984175e-04, -7.44427306e-04, -6.28096322e-01])

            # left
            fx=1.97562480e+03
            fy=1.97424657e+03
            cx=9.74580525e+02
            cy=4.24990872e+02
            distcoeffs = np.array([-0.55736824, 0.36627329, 0.00116797, -0.00214494, -0.28372033])


            camera_matrix=np.array([[fx,0,cx],[0,fy,cy],[0,0,1]])
            # distcoeffs=np.array([-0.61758395,-0.12767886,-0.00095697,0.00142091,0])

            optimalMat, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distcoeffs, (1920,1080), 1)
            newimage = np.array([])
            try:
                newimage=cv2.undistort(img,camera_matrix,distcoeffs,newimage,optimalMat)
            except:
                print("test")
            # cv2.imshow("undistort",newimage)
            # cv2.waitKey(0)
            cv2.imwrite(os.path.join(r"D:",each_file),newimage)
            # cv2.imwrite(os.path.join(r"D:", each_file).replace(".jpg", "_undistort.jpg"), newimage)
print("complete!")