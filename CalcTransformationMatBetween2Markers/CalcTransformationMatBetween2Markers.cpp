// CalcTransformationMatBetween2Markers.cpp : �R���\�[�� �A�v���P�[�V�����̃G���g�� �|�C���g���`���܂��B
//

#include "stdafx.h"

using namespace std;
using namespace cv;

#pragma warning(disable:4819)

#include <windows.h>
#include <stdio.h>
#include <stdlib.h>

#define _USE_MATH_DEFINES	// math.h��M_PI���g������
#include <math.h>			// �p�x�v�Z�p

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <AR/ar.h>
#include <AR/param.h>
#include <AR/video.h>
#include <AR/gsub.h>

// �O���[�o���ϐ�
/* �J�����\�� */
char *vconf_name = "Data/WDM_camera_flipV.xml";	// �r�f�I�f�o�C�X�̐ݒ�t�@�C��
int  xsize;											// �E�B���h�E�T�C�Y
int  ysize;											// �E�B���h�E�T�C�Y
int  thresh = 100;									// 2�l����臒l
int  countT = 0;										// �����t���[����

/* �J�����p�����[�^ */
const char *cparam_name = "Data/intrinsicParam.xml";			// �J�����p�����[�^�t�@�C��
//char *cparam_name = "Data/camera_para.dat";			// �J�����p�����[�^�t�@�C��
ARParam cparam;										// �J�����p�����[�^
static Mat cameraPara, distCoeffs;

static ARGL_CONTEXT_SETTINGS_REF gArglSettings = NULL;

/* �J��������̉摜�i�[�p(OpenCV) */
static Mat imageMat;
static VideoCapture cap(0);

/* �p�^�[���t�@�C�� */
#define MARK_NUM		2						// �g�p����}�[�J�[�̌�
//----- �f�B�X�v���C�ɐݒu����}�[�J
#define MARK1_MARK_ID	1						// �}�[�J�[ID
#define MARK1_PATT_NAME	"Data\\patt.hiro"		// �p�^�[���t�@�C����
#define MARK1_SIZE		183.0					// �p�^�[���̕��imm�j
//#define MARK1_SIZE		715.0					// �p�^�[���̕��imm�j
//----- �v�����g����Kinect�ł��B�e����}�[�J
#define MARK2_MARK_ID	2						// �}�[�J�[ID
#define MARK2_PATT_NAME	"Data\\patt.sample1"	// �p�^�[���t�@�C����
#define MARK2_SIZE		188.0					// �p�^�[���̕��imm
//#define MARK2_PATT_NAME	"Data\\patt.sample2"	// �p�^�[���t�@�C����
//#define MARK2_SIZE		673.0					// �p�^�[���̕��imm�j
//-----
//#define MARK3_MARK_ID	3						// �}�[�J�[ID
//#define MARK3_PATT_NAME	"Data\\patt.kanji"		// �p�^�[���t�@�C����
//#define MARK3_SIZE		80.0					// �p�^�[���̕��i80mm�j
//-----
typedef struct {
	char   *patt_name;			// �p�^�[���t�@�C��
	int    patt_id;				// �p�^�[����ID
	int    mark_id;				// �}�[�J�[ID
	int    visible;				// ���o�t���O
	double patt_width;			// �p�^�[���̃T�C�Y�i�P�ʁF�����j
	double patt_center[2];		// �p�^�[���̒��S���W
	double patt_trans[3][4];	// ���W�ϊ��s��
} MARK_T;
//-----
//MARK_T   marker[MARK_NUM] = {
//		{ MARK1_PATT_NAME, -1, MARK1_MARK_ID, 0, MARK1_SIZE, { 0.0, 0.0 } },
//		{ MARK2_PATT_NAME, -1, MARK2_MARK_ID, 0, MARK2_SIZE, { 0.0, 0.0 } },
//		{ MARK3_PATT_NAME, -1, MARK3_MARK_ID, 0, MARK3_SIZE, { 0.0, 0.0 } }
//};
MARK_T   marker[MARK_NUM] = {
		{ MARK1_PATT_NAME, -1, MARK1_MARK_ID, 0, MARK1_SIZE, { 0.0, 0.0 } },
		{ MARK2_PATT_NAME, -1, MARK2_MARK_ID, 0, MARK2_SIZE, { 0.0, 0.0 } }
};

// �v���g�^�C�v�錾
void Init(void);
void MainLoop(void);
void SetupLighting1(void);
void SetupLighting2(void);
void SetupMaterial1(void);
void SetupMaterial2(void);
void KeyEvent(unsigned char key, int x, int y);
void MouseEvent(int button, int state, int x, int y);
void Cleanup(void);
void DrawObject(int mark_id, double patt_trans[3][4]);


//=======================================================
// main�֐�
//=======================================================
int main(int argc, char **argv)
{
	// GLUT�̏�����
	glutInit(&argc, argv);

	// AR�A�v���P�[�V�����̏�����
	Init();

	// �r�f�I�L���v�`���̊J�n
	arVideoCapStart();

	// ���C�����[�v�̊J�n
	argMainLoop(MouseEvent, KeyEvent, MainLoop);

	return 0;
}

int arParamLoadCV(const char* cparam_name, ARParam* wparam)
{
	FileStorage cvfs(cparam_name, CV_STORAGE_READ);
	FileNode node(cvfs.fs, NULL);
	FileNode fn = node[string("mat_array")];
	read(fn[0], cameraPara);
	read(fn[1], distCoeffs);

	wparam->mat[0][0] = cameraPara.at<double>(0, 0);
	wparam->mat[0][1] = cameraPara.at<double>(0, 1);
	wparam->mat[0][2] = cameraPara.at<double>(0, 2);
	wparam->mat[0][3] = 0;
	wparam->mat[1][0] = cameraPara.at<double>(1, 0);
	wparam->mat[1][1] = cameraPara.at<double>(1, 1);
	wparam->mat[1][2] = cameraPara.at<double>(1, 2);
	wparam->mat[1][3] = 0;
	wparam->mat[2][0] = cameraPara.at<double>(2, 0);
	wparam->mat[2][1] = cameraPara.at<double>(2, 1);
	wparam->mat[2][2] = cameraPara.at<double>(2, 2);
	wparam->mat[2][3] = 0;

	wparam->dist_factor[0] = cameraPara.at<double>(0, 2);
	wparam->dist_factor[1] = cameraPara.at<double>(1, 2);
	wparam->dist_factor[2] = 1;// distCoeffs.at<double>(0, 0);
	wparam->dist_factor[3] = 1;

	wparam->xsize = xsize;
	wparam->ysize = ysize;

	return 0;
}

//=======================================================
// �������֐�
//=======================================================
void Init(void)
{
	ARParam wparam;		// �J�����p�����[�^

	// �r�f�I�f�o�C�X�̐ݒ�
	//if (arVideoOpen(vconf_name) < 0){
	//	printf("�r�f�I�f�o�C�X�̃G���[\n");
	//	exit(0);
	//}
	if (!cap.isOpened()) {
		cout << "error video" << endl;
	}

	// �E�B���h�E�T�C�Y�̎擾
	//if (arVideoInqSize(&xsize, &ysize) < 0) exit(0);
	xsize = cap.get(CV_CAP_PROP_FRAME_WIDTH);
	ysize = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	printf("Image size (x,y) = (%d,%d)\n", xsize, ysize);

	// �J�����p�����[�^�̐ݒ�
	//if (arParamLoad(cparam_name, 1, &wparam) < 0){
	if (arParamLoadCV(cparam_name, &wparam) < 0){
		printf("�J�����p�����[�^�̓ǂݍ��݂Ɏ��s���܂���\n");
		exit(0);
	}

	// �J�����p�����[�^�̃T�C�Y����
	arParamChangeSize(&wparam, xsize, ysize, &cparam);
	// �J�����p�����[�^�̏�����
	arInitCparam(&cparam);
	printf("*** Camera Parameter ***\n");
	arParamDisp(&cparam);

	// �p�^�[���t�@�C���̃��[�h
	for (int i = 0; i<MARK_NUM; i++){
		if ((marker[i].patt_id = arLoadPatt(marker[i].patt_name)) < 0){
			printf("�p�^�[���t�@�C���̓ǂݍ��݂Ɏ��s���܂���\n");
			printf("%s\n", marker[i].patt_name);
			exit(0);
		}
	}

	// gsub���C�u�����̏�����
	argInit(&cparam, 1.0, 0, 0, 0, 0);

	// Setup argl library for current context.
	
	if ((gArglSettings = arglSetupForCurrentContext()) == NULL) {
		fprintf(stderr, "main(): arglSetupForCurrentContext() returned error.\n");
		exit(-1);
	}
	

	// �E�B���h�E�^�C�g���̐ݒ�
	glutSetWindowTitle("Calc trans matrix");
}

ARUint8* cvVideoGetImage()
{
	Mat bufMat;
	cap >> bufMat;

	if (bufMat.empty()) { return nullptr; }

	// Undistort
	Mat bufMatUndist;
	undistort(bufMat, bufMatUndist, cameraPara, distCoeffs);
	
	cvtColor(bufMatUndist, imageMat, CV_BGR2BGRA);
	// ARUint8�ɃL���X�g
	ARUint8 *imageBuffer = reinterpret_cast<ARUint8*>(imageMat.data);

	return imageBuffer;
}

//=======================================================
// ���C�����[�v�֐�
//=======================================================
void MainLoop(void)
{
	ARUint8          *image;			// �J�����L���v�`���摜
	ARMarkerInfo     *marker_info;		// �}�[�J���
	int              marker_num;		// ���o���ꂽ�}�[�J�̐�
	int              i, j, k;

	// �J�����摜�̎擾
	if ((image = cvVideoGetImage()) == NULL){
	//if ((image = (ARUint8 *)arVideoGetImage()) == NULL){
		arUtilSleep(2);
		return;
	}
	if (countT == 0) arUtilTimerReset();
	countT++;

	// �J�����摜�̕`��
	arglDistortionCompensationSet(gArglSettings, FALSE);	// ARToolKit�c�ݕ␳���I�t��
	argDrawMode2D();
	argDispImage(image, 0, 0);

	// �}�[�J�̌��o�ƔF��
	if (arDetectMarker(image, thresh, &marker_info, &marker_num) < 0){
		Cleanup();
		exit(0);
	}

	// ���̉摜�̃L���v�`���w��
	arVideoCapNext();

	// 3D�I�u�W�F�N�g��`�悷�邽�߂̏���
	argDrawMode3D();
	argDraw3dCamera(0, 0);
	glClearDepth(1.0);					// �f�v�X�o�b�t�@�̏����l
	glClear(GL_DEPTH_BUFFER_BIT);		// �f�v�X�o�b�t�@�̏�����

	// �}�[�J�̈�v�x�̔�r
	for (i = 0; i<MARK_NUM; i++){
		k = -1;
		for (j = 0; j<marker_num; j++){
			if (marker[i].patt_id == marker_info[j].id){
				if (k == -1) k = j;
				else if (marker_info[k].cf < marker_info[j].cf) k = j;
			}
		}

		// �}�[�J�[��������Ȃ������Ƃ�
		if (k == -1){
			marker[i].visible = 0;
			continue;
		}

		cout << "confidence[" << i << "]: " << marker_info[k].cf << endl;

		// ���W�ϊ��s����擾
		if (marker[i].visible == 0) {
			// 1�t���[�����g���ă}�[�J�̈ʒu�E�p���i���W�ϊ��s��j�̌v�Z
			arGetTransMat(&marker_info[k], marker[i].patt_center, marker[i].patt_width, marker[i].patt_trans);
		}
		else {
			// �O�̃t���[�����g���ă}�[�J�̈ʒu�E�p���i���W�ϊ��s��j�̌v�Z
			arGetTransMatCont(&marker_info[k], marker[i].patt_trans, marker[i].patt_center, marker[i].patt_width, marker[i].patt_trans);
		}
		marker[i].visible = 1;

		// 3D�I�u�W�F�N�g�̕`��
		DrawObject(marker[i].mark_id, marker[i].patt_trans);
	}

	// �o�b�t�@�̓��e����ʂɕ\��
	argSwapBuffers();

	// 2�̃}�[�J�Ԃ̋�����\���i�}�[�J1[Hiro]�ƃ}�[�J2[Sample1]��F�������ꍇ�j
	if (marker[0].visible > 0 && marker[1].visible > 0){
		double wmat1[3][4], wmat2[3][4];

		// �r���[���}�[�J�s��i�J�������W�n����ɍl�����}�[�J�̈ʒu�E�p���j���擾
		arUtilMatInv(marker[0].patt_trans, wmat1);
		// �}�[�J1���W�n����ɍl�����}�[�J2�̈ʒu�E�p���i���}�[�J1�ƃ}�[�J2�̋����E�p�x�̍��j���擾
		arUtilMatMul(wmat1, marker[1].patt_trans, wmat2);

		// ������\��(x, y, z)
		//printf("%5.4lf[mm] %5.4lf[mm] %5.4lf[mm]\n", wmat2[0][3], wmat2[1][3], wmat2[2][3]);
	}

#if 0
	// 2�̃}�[�J�Ԃ̊p�x�̍���\���i�}�[�J1[Hiro]�ƃ}�[�J3[Kanji]��F�������ꍇ�j
	if (marker[0].visible > 0 && marker[2].visible > 0){
		double wmat1[3][4], wmat2[3][4];
		double yaw, pitch, roll;

		// �r���[���}�[�J�s��i�J�������W�n����ɍl�����}�[�J�̈ʒu�E�p���j���擾
		arUtilMatInv(marker[0].patt_trans, wmat1);
		// �}�[�J1���W�n����ɍl�����}�[�J3�̈ʒu�i���}�[�J1�ƃ}�[�J3�̋����E�p���̍��j���擾
		arUtilMatMul(wmat1, marker[2].patt_trans, wmat2);

		// �p���̍���\��
		//for( i=0; i<3; i++ ) {
		//    for( j=0; j< 3; j++ ) printf("%5.4f ", wmat2[i][j]);
		//    printf("\n");
		//}
		//printf("\n");

		// �p�x�̍���\���i-180���`180���j
		yaw = atan2(wmat2[1][0], wmat2[0][0]);
		pitch = atan2(wmat2[2][1], wmat2[2][2]);
		roll = atan2(wmat2[2][0], sqrt(wmat2[2][1] * wmat2[2][1] + wmat2[2][2] * wmat2[2][2]));

		printf("yaw = %4.4lf pitch = %4.4lf roll = %4.4lf\n", 180.0*yaw / M_PI, 180.0*pitch / M_PI, 180.0*roll / M_PI);
	}
#endif
}


//=======================================================
// 3D�I�u�W�F�N�g�̕`����s���֐�
//=======================================================
void DrawObject(int mark_id, double patt_trans[3][4])
{
	double gl_para[16];	// ARToolKit->OpenGL�ϊ��s��

	// �A�ʏ���
	glEnable(GL_DEPTH_TEST);			// �A�ʏ����E�L��
	glDepthFunc(GL_LEQUAL);			// �f�v�X�e�X�g

	// �ϊ��s��̓K�p
	argConvGlpara(patt_trans, gl_para);	// ARToolKit����OpenGL�̍s��ɕϊ�
	glMatrixMode(GL_MODELVIEW);			// �s��ϊ����[�h�E���f���r���[
	glLoadMatrixd(gl_para);				// �ǂݍ��ލs����w��

	switch (mark_id){
	case MARK1_MARK_ID:
		// ���C�e�B���O
		SetupLighting1();			// ���C�g�̒�`
		glEnable(GL_LIGHTING);	// ���C�e�B���O�E�L��
		glEnable(GL_LIGHT0);		// ���C�g0�E�I��
		// �I�u�W�F�N�g�̍ގ�
		SetupMaterial1();

		// 3D�I�u�W�F�N�g�̕`��
		glRotatef(90, 1, 0, 0);
		//glTranslatef(0.0, 0.0, 25.0);	// �}�[�J�̏�ɍڂ��邽�߂�Z�����i�}�[�J����j��25.0[mm]�ړ�
		glutSolidTeapot(100.0);			// �\���b�h�L���[�u��`��i1�ӂ̃T�C�Y50[mm]�j
		break;

	case MARK2_MARK_ID:
		// ���C�e�B���O
		SetupLighting2();			// ���C�g�̒�`
		glEnable(GL_LIGHTING);	// ���C�e�B���O�E�L��
		glEnable(GL_LIGHT0);		// ���C�g0�E�I��
		// �I�u�W�F�N�g�̍ގ�
		SetupMaterial2();

		// 3D�I�u�W�F�N�g�̕`��
		glRotatef(90, 1, 0, 0);
		//glTranslatef(0.0, 0.0, 50.0);		// �}�[�J�̏�ɍڂ��邽�߂�Z�����i�}�[�J����j��25.0[mm]�ړ�
		glutSolidTeapot(100.0);	// �\���b�h�X�t�B�A��`��i1�ӂ̃T�C�Y[mm]�j
		break;

	//case MARK3_MARK_ID:
	//	// ���C�e�B���O
	//	SetupLighting1();			// ���C�g�̒�`
	//	glEnable(GL_LIGHTING);	// ���C�e�B���O�E�L��
	//	glEnable(GL_LIGHT0);		// ���C�g0�E�I��
	//	// �I�u�W�F�N�g�̍ގ�
	//	SetupMaterial2();

	//	// 3D�I�u�W�F�N�g�̕`��
	//	glTranslatef(0.0, 0.0, 25.0);	// �}�[�J�̏�ɍڂ��邽�߂�Z�����i�}�[�J����j��25.0[mm]�ړ�
	//	glRotated(90, 1.0, 0.0, 0.0);	// �e�B�[�|�b�g���}�[�J��ɍڂ��邽�߂�90����]
	//	glutSolidTeapot(50.0);		// �\���b�h�e�B�[�|�b�g��`��i�T�C�Y50[mm]�j
		//break;
	}


	// �I������
	glDisable(GL_LIGHTING);		// ���C�e�B���O�E����
	glDisable(GL_DEPTH_TEST);		// �f�v�X�e�X�g�E����
}


//=======================================================
// ���C�e�B���O
//=======================================================
void SetupLighting1(void)
{
	// ���C�g�̒�`
	GLfloat lt0_position[] = { 100.0, -200.0, 200.0, 0.0 };	// ���C�g0�̈ʒu
	GLfloat lt0_ambient[] = { 0.1, 0.1, 0.1, 1.0 };			// �@�@�@�@ ����
	GLfloat lt0_diffuse[] = { 0.8, 0.8, 0.8, 1.0 };			// �@�@�@�@ �g�U��

	// ���C�g�̐ݒ�
	glLightfv(GL_LIGHT0, GL_POSITION, lt0_position);
	glLightfv(GL_LIGHT0, GL_AMBIENT, lt0_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lt0_diffuse);
}

void SetupLighting2(void)
{
	// ���C�g�̒�`
	GLfloat lt0_position[] = { 100.0, 200.0, 200.0, 0.0 };	// ���C�g0�̈ʒu
	GLfloat lt0_ambient[] = { 0.2, 0.2, 0.2, 1.0 };			// �@�@�@�@ ����
	GLfloat lt0_diffuse[] = { 0.8, 0.8, 0.8, 1.0 };			// �@�@�@�@ �g�U��

	// ���C�g�̐ݒ�
	glLightfv(GL_LIGHT0, GL_POSITION, lt0_position);
	glLightfv(GL_LIGHT0, GL_AMBIENT, lt0_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lt0_diffuse);
}


//=======================================================
// �}�e���A���̐ݒ�
//=======================================================
void SetupMaterial1(void)
{
	// �I�u�W�F�N�g�̍ގ�
	GLfloat mat_ambient[] = { 0.0, 1.0, 1.0, 1.0 };	// �ގ��̊���
	GLfloat mat_specular[] = { 0.0, 0.0, 1.0, 1.0 };	// ���ʌ�
	GLfloat mat_shininess[] = { 50.0 };				// ���ʌW��

	// �}�e���A���̐ݒ�
	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
}

void SetupMaterial2(void)
{
	// �I�u�W�F�N�g�̍ގ�
	GLfloat mat_ambient[] = { 0.0, 0.0, 1.0, 1.0 };	// �ގ��̊���
	GLfloat mat_specular[] = { 0.0, 0.0, 1.0, 1.0 };	// ���ʌ�
	GLfloat mat_shininess[] = { 50.0 };				// ���ʌW��

	// �}�e���A���̐ݒ�
	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
}

//=======================================================
// ���W�ϊ��s��ۑ��֐�
//=======================================================
void saveTMatrix()
{
	const char* transMatName = "SavedData\\T_Marker2Display.xml";

	// �eMarker to Camera�ϊ��s��
	Mat T1 = (Mat_<float>(4, 4) <<
		(float)marker[0].patt_trans[0][0], (float)marker[0].patt_trans[0][1], (float)marker[0].patt_trans[0][2], (float)marker[0].patt_trans[0][3],
		(float)marker[0].patt_trans[1][0], (float)marker[0].patt_trans[1][1], (float)marker[0].patt_trans[1][2], (float)marker[0].patt_trans[1][3],
		(float)marker[0].patt_trans[2][0], (float)marker[0].patt_trans[2][1], (float)marker[0].patt_trans[2][2], (float)marker[0].patt_trans[2][3],
		0.0f, 0.0f, 0.0f, 1.0f
		);
	Mat T2 = (Mat_<float>(4, 4) <<
		(float)marker[1].patt_trans[0][0], (float)marker[1].patt_trans[0][1], (float)marker[1].patt_trans[0][2], (float)marker[1].patt_trans[0][3],
		(float)marker[1].patt_trans[1][0], (float)marker[1].patt_trans[1][1], (float)marker[1].patt_trans[1][2], (float)marker[1].patt_trans[1][3],
		(float)marker[1].patt_trans[2][0], (float)marker[1].patt_trans[2][1], (float)marker[1].patt_trans[2][2], (float)marker[1].patt_trans[2][3],
		0.0f, 0.0f, 0.0f, 1.0f
		);

	// Marker2����Marker1�ւ̕ϊ��s������߂�
	Mat T = T1.inv() * T2;

	// ���W�ϊ��s���ۑ�
	FileStorage cvfs(transMatName, CV_STORAGE_WRITE);
	WriteStructContext ws(cvfs, "T_M2D", CV_NODE_SEQ);
	write(cvfs, "", T);
	cout << "Transformation matrix (Marker2 to Marker1) saved.\n" << T << endl;

}

//=======================================================
// �L�[�{�[�h���͏����֐�
//=======================================================
void KeyEvent(unsigned char key, int x, int y)
{
	// ESC�L�[����͂�����A�v���P�[�V�����I��
	if (key == 0x1b){
		printf("*** %f (frame/sec)\n", (double)countT / arUtilTimer());
		Cleanup();
		exit(0);
	}
	// �}�[�J���m�̍��W�ϊ��s���ۑ�
	else if (key == 's' || key == 'S'){
		if (marker[0].visible > 0 && marker[1].visible > 0){
			saveTMatrix();

			cout << "Press any key for quiting" << endl;
			getchar();
			Cleanup();
			exit(0);
		}
		else {
			printf("Couldn't find all markers, try again.\n");
		}
	}
}


//=======================================================
// �}�E�X���͏����֐�
//=======================================================
void MouseEvent(int button, int state, int x, int y)
{
	// ���͏�Ԃ�\��
	printf("�{�^���F%d ��ԁF%d ���W�F(x,y)=(%d,%d) \n", button, state, x, y);
}


//=======================================================
// �I�������֐�
//=======================================================
void Cleanup(void)
{
	arVideoCapStop();	// �r�f�I�L���v�`���̒�~
	arVideoClose();		// �r�f�I�f�o�C�X�̏I��
	argCleanup();		// ARToolKit�̏I������
}