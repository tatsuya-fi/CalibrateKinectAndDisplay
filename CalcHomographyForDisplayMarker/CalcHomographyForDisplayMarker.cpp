// CalcHomographyForDisplayMarker.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"

using namespace std;
using namespace cv;

const static float MARKER_LENGTH = 715;

const static char *windowName = "CalcHomography";
static vector<Point2f> calibPoints;

// ウィンドウの枠を消さない場合そのオフセットを考慮する
static int offsetX = 0;
static int offsetY = 0;

//setMouseCallbackへ渡すコールバック関数。
void mfunc(int event, int x, int y, int flags, void  *param){

	MouseParam *mparam = (MouseParam*)param;
	mparam->x = x;
	mparam->y = y;
	mparam->event = event;
	mparam->flags = flags;

	std::string desc;
	// マウスボタン，及び修飾キーを取得
	if (flags & cv::EVENT_FLAG_LBUTTON)
		desc += " + LBUTTON";
	if (flags & cv::EVENT_FLAG_RBUTTON)
		desc += " + RBUTTON";
	if (flags & cv::EVENT_FLAG_MBUTTON)
		desc += " + MBUTTON";
	if (flags & cv::EVENT_FLAG_CTRLKEY)
		desc += " + CTRL";
	if (flags & cv::EVENT_FLAG_SHIFTKEY)
		desc += " + SHIFT"; 
	if (flags & cv::EVENT_FLAG_ALTKEY)
		desc += " + ALT";

	//std::cout << desc << " (" << x << ", " << y << ")" << std::endl;
	if (flags & cv::EVENT_FLAG_LBUTTON)
	{
		int mx = x + offsetX;
		int my = y + offsetY;
		for (int i = 0; i < calibPoints.size(); ++i)
		{
			if (sqrt(pow(calibPoints[i].x - mx, 2) + pow(calibPoints[i].y - my, 2)) < 10)
				return;
		}
		Point2f newPoint(mx, my);
		calibPoints.push_back(newPoint);
		cout << "(" << mx << ", " << my << ")" << endl;

		waitKey(50);
	}
}

int _tmain(int argc, _TCHAR* argv[])
{
	int mainDisplayWidth;
	int mainDisplayHeight;


	// ウィンドウの作成
	namedWindow(windowName, WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO);
	moveWindow(windowName, 0, 0);

	// マウスイベントの登録
	MouseParam mparam;
	mparam.x = 0; mparam.y = 0; mparam.event = 0; mparam.flags = 0;
	setMouseCallback(windowName, &mfunc, &mparam);
	
#if 0
	// windowNameを持つウィンドウを検索
	HWND windowHandle = ::FindWindowA(NULL, windowName);

	// ウィンドウを最大化する（枠も消す）
	if (NULL != windowHandle) {

		//// ウィンドウスタイル変更（メニューバーなし、最前面）
		SetWindowLongPtr(windowHandle, GWL_STYLE, WS_POPUP);
		SetWindowLongPtr(windowHandle, GWL_EXSTYLE, WS_EX_TOPMOST);

		//// 最大化する
		ShowWindow(windowHandle, SW_MAXIMIZE);
		//setWindowProperty(windowName, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
		cvSetWindowProperty(windowName, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

		// ディスプレイサイズを取得
		mainDisplayWidth = GetSystemMetrics(SM_CXSCREEN);
		mainDisplayHeight = GetSystemMetrics(SM_CYSCREEN);
		cout << "Display Size: " << mainDisplayWidth << ", " << mainDisplayHeight << endl;

		//// クライアント領域をディスプレーに合わせる
		SetWindowPos(windowHandle, NULL,
			0, 0, mainDisplayWidth * 2
			, mainDisplayWidth,
			SWP_FRAMECHANGED | SWP_NOZORDER);
	}
#else
	// 使用するウィンドウすべての合計サイズ
	mainDisplayWidth = 7580;
	mainDisplayHeight = 974;
	//mainDisplayWidth = 5120;
	//mainDisplayHeight = 1024;

#endif
	cout << "左上から時計回りにクリック" << endl;

	Mat img = Mat::ones(mainDisplayHeight, mainDisplayWidth, CV_8UC3);
	while (1)
	{
		Scalar color;
		calibPoints.size() == 4 ? color = Scalar(0, 255, 0) : color = Scalar(0, 0, 255);
		for (int i = 0; i < calibPoints.size(); ++i)
		{
			circle(img, Point(calibPoints[i].x , calibPoints[i].y), 2, color, -1);
		}
		
		imshow(windowName, img);

		if (calibPoints.size() == 4) { break; }
		else if (calibPoints.size() > 4)
		{
			cout << "something wrong." << endl;
			return -1;
		}

		int key = cv::waitKey(20);
		// 'Esc'が押された場合に終了
		if (key == 27) return 0;
	}

	vector<Point2f>markerPoints = {
			{ -MARKER_LENGTH / 2, MARKER_LENGTH / 2 },
			{  MARKER_LENGTH / 2, MARKER_LENGTH / 2 },
			{  MARKER_LENGTH / 2,-MARKER_LENGTH / 2 },
			{ -MARKER_LENGTH / 2,-MARKER_LENGTH / 2 },
	};

	Mat H = findHomography(markerPoints, calibPoints);

	Mat a = (Mat_<double>(3, 1) << 0, 0, 1);
	a = H * a;
	a /= a.at<double>(2, 0);
	circle(img, Point((int)a.at<double>(0, 0), (int)a.at<double>(1, 0)), 3, Scalar(0, 0, 255), 2);
	cout << a << endl;
	Mat pt0 = (Mat_<double>(3, 1) << -MARKER_LENGTH / 2, MARKER_LENGTH / 2, 1);
	pt0 = H * pt0;
	pt0 /= pt0.at<double>(2, 0);
	Mat pt1 = (Mat_<double>(3, 1) << MARKER_LENGTH / 2, MARKER_LENGTH / 2, 1);
	pt1 = H * pt1;
	pt1 /= pt1.at<double>(2, 0);
	Mat pt2 = (Mat_<double>(3, 1) << MARKER_LENGTH / 2, -MARKER_LENGTH / 2, 1);
	pt2 = H * pt2;
	pt2 /= pt2.at<double>(2, 0);
	Mat pt3 = (Mat_<double>(3, 1) << -MARKER_LENGTH / 2, -MARKER_LENGTH / 2, 1);
	pt3 = H * pt3;
	pt3 /= pt3.at<double>(2, 0);
	circle(img, Point((int)a.at<double>(0, 0)-offsetX, (int)a.at<double>(1, 0)-offsetY), 3, Scalar(0, 0, 255), 2);

	line(img, Point((int)pt0.at<double>(0, 0)-offsetX, (int)pt0.at<double>(1, 0)-offsetY), Point((int)pt1.at<double>(0, 0)-offsetX, (int)pt1.at<double>(1, 0)-offsetY), Scalar(0, 0, 255), 2);
	line(img, Point((int)pt1.at<double>(0, 0) - offsetX, (int)pt1.at<double>(1, 0) - offsetY), Point((int)pt2.at<double>(0, 0) - offsetX, (int)pt2.at<double>(1, 0) - offsetY), Scalar(0, 0, 255), 2);
	line(img, Point((int)pt2.at<double>(0, 0)-offsetX, (int)pt2.at<double>(1, 0)-offsetY), Point((int)pt3.at<double>(0, 0)-offsetX, (int)pt3.at<double>(1, 0)-offsetY), Scalar(0, 0, 255), 2);
	line(img, Point((int)pt0.at<double>(0, 0)-offsetX, (int)pt0.at<double>(1, 0)-offsetY), Point((int)pt3.at<double>(0, 0)-offsetX, (int)pt3.at<double>(1, 0)-offsetY), Scalar(0, 0, 255), 2);
	line(img, Point((int)pt2.at<double>(0, 0)-offsetX, (int)pt2.at<double>(1, 0)-offsetY), Point((int)pt0.at<double>(0, 0)-offsetX, (int)pt0.at<double>(1, 0)-offsetY), Scalar(0, 0, 255), 2);
	line(img, Point((int)pt1.at<double>(0, 0)-offsetX, (int)pt1.at<double>(1, 0)-offsetY), Point((int)pt3.at<double>(0, 0)-offsetX, (int)pt3.at<double>(1, 0)-offsetY), Scalar(0, 0, 255), 2);
	imshow(windowName, img);

	FileStorage cvfs("output/H.xml", CV_STORAGE_WRITE);
	write(cvfs, "disp1", H);

	cout << "calibration finished" << endl;
	cout << H << endl;
	cout << "Click any key to quit..." << endl;
	waitKey();

	return 0;
}

