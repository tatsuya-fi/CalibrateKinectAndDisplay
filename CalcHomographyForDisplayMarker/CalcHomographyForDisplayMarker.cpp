// CalcHomographyForDisplayMarker.cpp : �R���\�[�� �A�v���P�[�V�����̃G���g�� �|�C���g���`���܂��B
//

#include "stdafx.h"

using namespace std;
using namespace cv;

const static float MARKER_LENGTH = 189;


const static char *windowName = "CalcHomography";
static vector<Point2f> calibPoints;

//setMouseCallback�֓n���R�[���o�b�N�֐��B
void mfunc(int event, int x, int y, int flags, void  *param){

	MouseParam *mparam = (MouseParam*)param;
	mparam->x = x;
	mparam->y = y;
	mparam->event = event;
	mparam->flags = flags;

	std::string desc;
	// �}�E�X�{�^���C�y�яC���L�[���擾
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
		for (int i = 0; i < calibPoints.size(); ++i)
		{
			if (sqrt(pow(calibPoints[i].x - x, 2) + pow(calibPoints[i].y - y, 2)) < 10)
				return;
		}
		Point2f newPoint(x, y);
		calibPoints.push_back(newPoint);
		cout << "(" << x << ", " << y << ")" << endl;
	}
}

int _tmain(int argc, _TCHAR* argv[])
{
	int mainDisplayWidth;
	int mainDisplayHeight;


	// �E�B���h�E�̍쐬
	namedWindow(windowName, WINDOW_AUTOSIZE);
	// �}�E�X�C�x���g�̓o�^
	MouseParam mparam;
	mparam.x = 0; mparam.y = 0; mparam.event = 0; mparam.flags = 0;
	setMouseCallback(windowName, &mfunc, &mparam);
	

	// windowName�����E�B���h�E������
	HWND windowHandle = ::FindWindowA(NULL, windowName);

	// �E�B���h�E���ő剻����i�g�������j
	if (NULL != windowHandle) {

		// �E�B���h�E�X�^�C���ύX�i���j���[�o�[�Ȃ��A�őO�ʁj
		SetWindowLongPtr(windowHandle, GWL_STYLE, WS_POPUP);
		SetWindowLongPtr(windowHandle, GWL_EXSTYLE, WS_EX_TOPMOST);

		// �ő剻����
		ShowWindow(windowHandle, SW_MAXIMIZE);
		setWindowProperty(windowName, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
		//cvSetWindowProperty(windowName, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

		// �f�B�X�v���C�T�C�Y���擾
		mainDisplayWidth = GetSystemMetrics(SM_CXSCREEN);
		mainDisplayHeight = GetSystemMetrics(SM_CYSCREEN);
		cout << "Display Size: " << mainDisplayWidth << ", " << mainDisplayHeight << endl;

		// �N���C�A���g�̈���f�B�X�v���[�ɍ��킹��
		SetWindowPos(windowHandle, NULL,
			0, 0, mainDisplayWidth, mainDisplayWidth,
			SWP_FRAMECHANGED | SWP_NOZORDER);
	}

	cout << "���ォ�玞�v���ɃN���b�N" << endl;

	Mat img = Mat::ones(mainDisplayHeight, mainDisplayWidth, CV_8UC3);
	while (1)
	{
		circle(img, Point(609, 444), 3, Scalar(0, 0, 255));

		Scalar color;
		calibPoints.size() == 4 ? color = Scalar(0, 0, 255) : color = Scalar(0, 255, 0);
		for (int i = 0; i < calibPoints.size(); ++i)
		{
			circle(img, Point(calibPoints[i].x, calibPoints[i].y), 2, color, -1);
		}
		imshow(windowName, img);
		
		if (calibPoints.size() == 4) { break; }
		else if (calibPoints.size() > 4)
		{
			cout << "something wrong." << endl;
			return -1;
		}
		
		int key = cv::waitKey(20);
		// 'Esc'�������ꂽ�ꍇ�ɏI��
		if (key == 27) return 0;
	}

	vector<Point2f>markerPoints = {
			{ -MARKER_LENGTH / 2, MARKER_LENGTH / 2 },
			{  MARKER_LENGTH / 2, MARKER_LENGTH / 2 },
			{  MARKER_LENGTH / 2,-MARKER_LENGTH / 2 },
			{ -MARKER_LENGTH / 2,-MARKER_LENGTH / 2 },
	};

	Mat H = findHomography(markerPoints, calibPoints);

	FileStorage cvfs("output/H.xml", CV_STORAGE_WRITE);
	write(cvfs, "disp1", H);

	cout << "calibration finished" << endl;
	cout << H << endl;
	waitKey();

	return 0;
}

