//  ����ͼ��������μ��.
//  Author��yao yongxiang
//  Time��2020.02.16
//  vs2015 C++ and OpenCV3.4.1

#include<opencv2/opencv.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>

#include<math.h>
#include<string.h>
#include<direct.h>//���������ļ�Ŀ¼
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
	// *************1 ************��ȡͼƬ//
	/*src = imread("33.png");
	if (!src.data)
	{
		printf("could not load image...");
		return -1;
	}*/

	// ��ȡ��Ƶ
	VideoCapture capture;
	capture.open("C:\\Users\\Administrator\\Desktop\\2020���ٹ����ļ�\\Ҧ����\\VID_20200219_114234.mp4");

	//ѭ����ʾÿһ֡
	int Num = 0;
	while (capture.isOpened())
	{

		Mat src;  //����һ��Mat�����frame�����ڴ洢ÿһ֡��ͼ��

		capture >> src;  //��ȡ��ǰ֡

	
		//�����ļ��б���Ӱ��
		char* newfile = ".\\save_image";
		_mkdir(newfile);
		namedWindow(INput, CV_WINDOW_AUTOSIZE);
		imshow(INput, src);
		//cout << "��1��������������" << src.size() << endl;

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

//************************�����μ��Ĵ���***************************
/*  �Լ�⵽�ľ��ν����ж�   */
static double angle(Point pt1, Point pt2, Point pt0) {
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	double ratio;//�߳�ƽ���ı�
	ratio = (dx1*dx1 + dy1 * dy1) / (dx2*dx2 + dy2 * dy2);
	if (ratio < 0.8 || 1.2 < ratio) {//���ݱ߳�ƽ���ıȹ�С�������ǰ��̭����ı��Σ������̭���࣬�����˱�������
									//      Log("ratio\n");
		return 1.0;//���ݱ߳�ƽ���ıȹ�С�������ǰ��̭����ı���
	}
	return (dx1*dx2 + dy1 * dy2) / sqrt((dx1*dx1 + dy1 * dy1)*(dx2*dx2 + dy2 * dy2) + 1e-10);
}

/*  ���ؼ�⵽��������   */
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
				approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);//0.02Ϊ��ë����ֱ��ϵ���������ë��������©�죬�����Ե���

				if (approx.size() == 4 && isContourConvex(Mat(approx))) {
					double area;
					area = fabs(contourArea(Mat(approx)));
					if (4000.0 < area && area < 30000.0) {
						double maxCosine = 0.0;

						for (int j = 2; j < 5; j++) {
							double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
							maxCosine = MAX(maxCosine, cosine);
							if (maxCosine == 1.0) break;// //�߳��ȳ����趨��Χ
						}
						if (maxCosine < 0.1) {
							squares.push_back(approx);
							return;   //��⵽һ���ϸ�������ξͷ���

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
