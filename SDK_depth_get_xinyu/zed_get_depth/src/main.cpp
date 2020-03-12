//  用于图像的正方形检测.
//  Author：yao yongxiang
//  Time：2020.02.16
//  vs2015 C++ and OpenCV3.4.1

#include<opencv2/opencv.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>

#include<math.h>
#include<string.h>
#include<direct.h>//遍历生成文件目录
#include<iostream>

using namespace std;
using namespace cv;

Mat src;
int thresh = 50;
int N = 5;

const char*  INput = "input image";
const char*  wndname = "Square Detection Demo";
static double angle(Point pt1, Point pt2, Point pt0);
void findSquares(const Mat& image, vector<vector<Point> >& squares);
static void drawSquares(Mat& image, const vector<vector<Point> >& squares);

int main()
{
	// *************1 ************获取图片//
	/*src = imread("33.png");
	if (!src.data)
	{
		printf("could not load image...");
		return -1;
	}*/

	// 读取视频
	VideoCapture capture;
	capture.open("C:\\Users\\Administrator\\Desktop\\2020寒假工作文件\\姚永祥\\VID_20200219_114234.mp4");

	//循环显示每一帧
	int Num = 0;
	while (capture.isOpened())
	{

		Mat src;  //定义一个Mat类变量frame，用于存储每一帧的图像

		capture >> src;  //读取当前帧

	
		//创建文件夹保存影像
		char* newfile = ".\\save_image";
		_mkdir(newfile);
		namedWindow(INput, CV_WINDOW_AUTOSIZE);
		imshow(INput, src);
		//cout << "第1步输出结果结束！" << src.size() << endl;

		vector<vector<Point>> squares;
		findSquares(src, squares);
		drawSquares(src, squares);
		imshow("wndname", src);
		string path = ".\\save_image\\" + to_string(Num) + ".jpg";
		imwrite(path, src);

		waitKey(1);
		

	}

	return 0;
}

//************************正方形检测的代码***************************
/*  对检测到的矩形进行判断   */
static double angle(Point pt1, Point pt2, Point pt0) {
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	double ratio;//边长平方的比
	ratio = (dx1*dx1 + dy1 * dy1) / (dx2*dx2 + dy2 * dy2);
	if (ratio < 0.8 || 1.2 < ratio) {//根据边长平方的比过小或过大提前淘汰这个四边形，如果淘汰过多，调整此比例数字
									//      Log("ratio\n");
		return 1.0;//根据边长平方的比过小或过大提前淘汰这个四边形
	}
	return (dx1*dx2 + dy1 * dy2) / sqrt((dx1*dx1 + dy1 * dy1)*(dx2*dx2 + dy2 * dy2) + 1e-10);
}

/*  返回检测到的正方形   */
void findSquares(const Mat& image, vector<vector<Point> >& squares) {
	squares.clear();
	Mat timg(image);
	medianBlur(image, timg, 9);
	Mat gray0(timg.size(), CV_8U), gray1;
	vector<vector<Point> > contours;
	// try several threshold levels
	for (int c = 0; c < 3; c++)
	{
		int ch[] = { c, 0 };
		mixChannels(&timg, 1, &gray0, 1, ch, 1);

		for (int l = 0; l < N; l++)
		{
			if (l == 0)
			{
				Canny(gray0, gray1, 5, thresh, 5);
				dilate(gray1, gray1, Mat(), Point(-1, -1));
			}
			else
			{
				gray1 = gray0 >= (l + 1) * 255 / N;
			}

			findContours(gray1, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

			vector<Point> approx;
			// test each contour
			for (size_t i = 0; i < contours.size(); i++) {
				approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);//0.02为将毛边拉直的系数，如果对毛边正方形漏检，可试试调大

				if (approx.size() == 4 && isContourConvex(Mat(approx))) {
					double area;
					area = fabs(contourArea(Mat(approx)));
					if (4000.0 < area && area < 30000.0) {
						double maxCosine = 0.0;

						for (int j = 2; j < 5; j++) {
							double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
							maxCosine = MAX(maxCosine, cosine);
							if (maxCosine == 1.0) break;// //边长比超过设定范围
						}
						if (maxCosine < 0.1) {
							squares.push_back(approx);
							return;   //检测到一个合格的正方形就返回

						}
					}
				}
			}
		}
	}
}


static void drawSquares(Mat& image, const vector<vector<Point> >& squares)
{
	for (size_t i = 0; i < squares.size(); i++)
	{
		const Point* p = &squares[i][0];

		int n = (int)squares[i].size();
		//dont detect the border
		if (p->x > 3 && p->y > 3)
			polylines(image, &p, &n, 1, true, Scalar(0, 0, 255), 3, LINE_AA);
	}
	imshow(wndname, image);
}
