// ARToolKitForKinectV2.cpp
//
 

#include "stdafx.h"

using namespace std;
using namespace cv;

//int _tmain(int argc, _TCHAR* argv[])
//{
//	KinectV2Basics app;
//	app.SetupKinectV2();
//	while (1)
//	{
//		//// Depth
//		Mat depthFrame, distanceMat;
//		Mat pointsMat;
//		if (app.GetDepthMat(depthFrame, distanceMat))
//		{
//			app.GetPointsMat(pointsMat);
//		}
//		if (!depthFrame.empty())
//			imshow("depth", depthFrame);
//		
//		// Color
//		Mat colorMat;
//		if (app.GetColorMat(colorMat, 0.5))
//			imshow("color", colorMat);
//
//		// Quit
//		int key = waitKey(20);
//		if (key == 'q' || key == VK_ESCAPE)
//		{
//			break;
//		}
//
//	}
//
//	return 0;
//}


// ============================================================================
//	Includes
// ============================================================================

#include <stdio.h>
#include <stdlib.h>					// malloc(), free()
#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif
#include <AR/config.h>
#include <AR/video.h>
#include <AR/param.h>			// arParamDisp()
#include <AR/ar.h>
#include <AR/gsub_lite.h>

// ============================================================================
//	Constants
// ============================================================================

#define VIEW_SCALEFACTOR		0.025		// 1.0 ARToolKit unit becomes 0.025 of my OpenGL units.
#define VIEW_DISTANCE_MIN		0.1			// Objects closer to the camera than this will not be displayed.
#define VIEW_DISTANCE_MAX		100.0		// Objects further away from the camera than this will not be displayed.

// ============================================================================
//	Global variables
// ============================================================================

// Kinect
static KinectV2Basics kinectApp;	// グローバルに定義するのは良くないとわかりつつも...!

// Preferences.
static int prefWindowed = TRUE;
static int prefWidth;					// Fullscreen mode width.
static int prefHeight;				// Fullscreen mode height.
static int prefDepth = 32;					// Fullscreen mode bit depth.
static int prefRefresh = 0;					// Fullscreen mode refresh rate. Set to 0 to use default rate.

// Image acquisition.
static ARUint8		*gARTImage = NULL;
static Mat			gARTImageMat;		// staticで定義しないと勝手に解放されてしまう

// Marker detection.
static int			gARTThreshhold = 100;
static long			gCallCountMarkerDetect = 0;

// Transformation matrix retrieval.
static double		gPatt_width = 188.5;	// Per-marker, but we are using only 1 marker.
static double		gPatt_centre[2] = { 0.0, 0.0 }; // Per-marker, but we are using only 1 marker.
static double		gPatt_trans[3][4];		// Per-marker, but we are using only 1 marker.
static int			gPatt_found = FALSE;	// Per-marker, but we are using only 1 marker.
static int			gPatt_id;				// Per-marker, but we are using only 1 marker.

// Drawing.
static ARParam		gARTCparam;
static ARGL_CONTEXT_SETTINGS_REF gArglSettings = NULL;
static int gDrawRotate = FALSE;
static float gDrawRotateAngle = 0;			// For use in drawing.

// ============================================================================
//	Functions
// ============================================================================

// Something to look at, draw a rotating colour cube.
static void DrawCube(void)
{
	// Colour cube data.
	static GLuint polyList = 0;
	float fSize = 0.5f;
	long f, i;
	const GLfloat cube_vertices[8][3] = {
			{ 1.0, 1.0, 1.0 }, { 1.0, -1.0, 1.0 }, { -1.0, -1.0, 1.0 }, { -1.0, 1.0, 1.0 },
			{ 1.0, 1.0, -1.0 }, { 1.0, -1.0, -1.0 }, { -1.0, -1.0, -1.0 }, { -1.0, 1.0, -1.0 } };
	const GLfloat cube_vertex_colors[8][3] = {
			{ 1.0, 1.0, 1.0 }, { 1.0, 1.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 0.0, 1.0, 1.0 },
			{ 1.0, 0.0, 1.0 }, { 1.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 1.0 } };
	GLint cube_num_faces = 6;
	const short cube_faces[6][4] = {
			{ 3, 2, 1, 0 }, { 2, 3, 7, 6 }, { 0, 1, 5, 4 }, { 3, 0, 4, 7 }, { 1, 2, 6, 5 }, { 4, 5, 6, 7 } };

	if (!polyList) {
		polyList = glGenLists(1);
		glNewList(polyList, GL_COMPILE);
		glBegin(GL_QUADS);
		for (f = 0; f < cube_num_faces; f++)
			for (i = 0; i < 4; i++) {
			glColor3f(cube_vertex_colors[cube_faces[f][i]][0], cube_vertex_colors[cube_faces[f][i]][1], cube_vertex_colors[cube_faces[f][i]][2]);
			glVertex3f(cube_vertices[cube_faces[f][i]][0] * fSize, cube_vertices[cube_faces[f][i]][1] * fSize, cube_vertices[cube_faces[f][i]][2] * fSize);
			}
		glEnd();
		glColor3f(0.0, 0.0, 0.0);
		for (f = 0; f < cube_num_faces; f++) {
			glBegin(GL_LINE_LOOP);
			for (i = 0; i < 4; i++)
				glVertex3f(cube_vertices[cube_faces[f][i]][0] * fSize, cube_vertices[cube_faces[f][i]][1] * fSize, cube_vertices[cube_faces[f][i]][2] * fSize);
			glEnd();
		}
		glEndList();
	}

	glPushMatrix(); // Save world coordinate system.
	glTranslatef(0.0, 0.0, 0.5); // Place base of cube on marker surface.
	glRotatef(gDrawRotateAngle, 0.0, 0.0, 1.0); // Rotate about z axis.
	glDisable(GL_LIGHTING);	// Just use colours.
	glCallList(polyList);	// Draw the cube.
	glPopMatrix();	// Restore world coordinate system.

}

static void DrawCubeUpdate(float timeDelta)
{
	if (gDrawRotate) {
		gDrawRotateAngle += timeDelta * 45.0f; // Rotate cube at 45 degrees per second.
		if (gDrawRotateAngle > 360.0f) gDrawRotateAngle -= 360.0f;
	}
}

static int cvParamLoad(const char* cname, ARParam* wparam)
{
	Mat cameraPara, distCoeffs;

	FileStorage cvfs(cname, CV_STORAGE_READ);
	FileNode node(cvfs.fs, NULL);
	read(node["cameraMatrix"], cameraPara);
	read(node["distCoeffs"], distCoeffs);

	cout << cameraPara << endl;
	cout << distCoeffs << endl;
	
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

	wparam->dist_factor[0] = cameraPara.at<double>(0, 0);
	wparam->dist_factor[1] = cameraPara.at<double>(1, 1);
	wparam->dist_factor[2] = distCoeffs.at<double>(0, 0);
	wparam->dist_factor[3] = 1;

	wparam->xsize = kinectApp.widthColor;
	wparam->ysize = kinectApp.heightColor;

	return (TRUE);
}

static int setupKinectV2ForARToolKit(const char *cparam_name, char *vconf, ARParam *cparam)
{
	ARParam			wparam;
	int				xsize, ysize;
	
	// Open the video path.
	//if (arVideoOpen(vconf) < 0) {
	//	fprintf(stderr, "setupCamera(): Unable to open connection to camera.\n");
	//	return (FALSE);
	//}
	if (!kinectApp.SetupKinectV2()) {
		fprintf(stderr, "setupKinectV2ForARToolKit(): Unable to set up Kinect v2 camera.\n");
		return (FALSE);
	}

	// Find the size of the window.
	//if (arVideoInqSize(&xsize, &ysize) < 0) return (FALSE);
	xsize = kinectApp.widthColor;
	ysize = kinectApp.heightColor;
	fprintf(stdout, "Camera image size (x,y) = (%d,%d)\n", xsize, ysize);

	// Load the camera parameters, resize for the window and init.
	if (cvParamLoad(cparam_name, &wparam) < 0) {
	//if (arParamLoad(cparam_name, 1, &wparam) < 0) {
		fprintf(stderr, "setupKinectV2ForARToolKit(): Error loading parameter file %s for camera.\n", cparam_name);
		return (FALSE);
	}
	arParamChangeSize(&wparam, xsize, ysize, cparam);
	fprintf(stdout, "*** Camera Parameter ***\n");
	arParamDisp(cparam);

	arInitCparam(cparam);

	//if (arVideoCapStart() != 0) {
	//	fprintf(stderr, "setupCamera(): Unable to begin camera data capture.\n");
	//	return (FALSE);
	//}

	return (TRUE);
}

static int setupMarker(const char *patt_name, int *patt_id)
{

	if ((*patt_id = arLoadPatt(patt_name)) < 0) {
		fprintf(stderr, "setupMarker(): pattern load error !!\n");
		return (FALSE);
	}

	return (TRUE);
}

// Report state of ARToolKit global variables arFittingMode,
// arImageProcMode, arglDrawMode, arTemplateMatchingMode, arMatchingPCAMode.
static void debugReportMode(void)
{
	if (arFittingMode == AR_FITTING_TO_INPUT) {
		fprintf(stderr, "FittingMode (Z): INPUT IMAGE\n");
	}
	else {
		fprintf(stderr, "FittingMode (Z): COMPENSATED IMAGE\n");
	}

	if (arImageProcMode == AR_IMAGE_PROC_IN_FULL) {
		fprintf(stderr, "ProcMode (X)   : FULL IMAGE\n");
	}
	else {
		fprintf(stderr, "ProcMode (X)   : HALF IMAGE\n");
	}

	if (arglDrawModeGet(gArglSettings) == AR_DRAW_BY_GL_DRAW_PIXELS) {
		fprintf(stderr, "DrawMode (C)   : GL_DRAW_PIXELS\n");
	}
	else if (arglTexmapModeGet(gArglSettings) == AR_DRAW_TEXTURE_FULL_IMAGE) {
		fprintf(stderr, "DrawMode (C)   : TEXTURE MAPPING (FULL RESOLUTION)\n");
	}
	else {
		fprintf(stderr, "DrawMode (C)   : TEXTURE MAPPING (HALF RESOLUTION)\n");
	}

	if (arTemplateMatchingMode == AR_TEMPLATE_MATCHING_COLOR) {
		fprintf(stderr, "TemplateMatchingMode (M)   : Color Template\n");
	}
	else {
		fprintf(stderr, "TemplateMatchingMode (M)   : BW Template\n");
	}

	if (arMatchingPCAMode == AR_MATCHING_WITHOUT_PCA) {
		fprintf(stderr, "MatchingPCAMode (P)   : Without PCA\n");
	}
	else {
		fprintf(stderr, "MatchingPCAMode (P)   : With PCA\n");
	}
}

static void Quit(void)
{
	arglCleanup(gArglSettings);
	arVideoCapStop();
	arVideoClose();
	exit(0);
}

static void saveTransMat()
{
	const char* transMatName = "SavedData\\T_Kinect2Marker.xml";
	if (gPatt_found) {
		Mat T_Marker2KinectCCamera = (Mat_<double>(4, 4) <<
			gPatt_trans[0][0], gPatt_trans[0][1], gPatt_trans[0][2], gPatt_trans[0][3],
			gPatt_trans[1][0], gPatt_trans[1][1], gPatt_trans[1][2], gPatt_trans[1][3],
			gPatt_trans[2][0], gPatt_trans[2][1], gPatt_trans[2][2], gPatt_trans[2][3],
			0, 0, 0, 1
		);
		FileStorage cvfs(transMatName, CV_STORAGE_WRITE);
		write(cvfs, "T_K2M", T_Marker2KinectCCamera.inv());

		cout << "Transformation matrix (Kinect color camera to Marker) saved.\n" << T_Marker2KinectCCamera.inv() << endl;
	}
	else {
		cout << "Marker was not found. Please try again." << endl;
	}
}

static void Keyboard(unsigned char key, int x, int y)
{
	int mode;
	switch (key) {
	case 0x1B:						// Quit.
	case 'Q':
	case 'q':
		Quit();
		break;
	case ' ':
		gDrawRotate = !gDrawRotate;
		break;
	case 'C':
	case 'c':
		mode = arglDrawModeGet(gArglSettings);
		if (mode == AR_DRAW_BY_GL_DRAW_PIXELS) {
			arglDrawModeSet(gArglSettings, AR_DRAW_BY_TEXTURE_MAPPING);
			arglTexmapModeSet(gArglSettings, AR_DRAW_TEXTURE_FULL_IMAGE);
		}
		else {
			mode = arglTexmapModeGet(gArglSettings);
			if (mode == AR_DRAW_TEXTURE_FULL_IMAGE)	arglTexmapModeSet(gArglSettings, AR_DRAW_TEXTURE_HALF_IMAGE);
			else arglDrawModeSet(gArglSettings, AR_DRAW_BY_GL_DRAW_PIXELS);
		}
		fprintf(stderr, "*** Camera - %f (frame/sec)\n", (double)gCallCountMarkerDetect / arUtilTimer());
		gCallCountMarkerDetect = 0;
		arUtilTimerReset();
		debugReportMode();
		break;
	case 'D':
	case 'd':
		arDebug = !arDebug;
		break;
	case '?':
	case '/':
		printf("Keys:\n");
		printf(" q or [esc]    Quit demo.\n");
		printf(" c             Change arglDrawMode and arglTexmapMode.\n");
		printf(" d             Activate / deactivate debug mode.\n");
		printf(" ? or /        Show this help.\n");
		printf(" s		       Save the transformation matrix from marker coordinate to Kinect camera coordinate.\n");
		printf("\nAdditionally, the ARVideo library supplied the following help text:\n");
		arVideoDispOption();
		break;

	case 's':
	case 'S':
		saveTransMat();

		break;
	default:
		break;
	}
}

static ARUint8* getKinectImage()
{
	// KinectからMat形式のカラー画像を取得
	Mat matCV;
	if (!kinectApp.GetColorMat(matCV))
	{
		return NULL;
	}
	//imshow("getKinectImage()", matCV);

	// メモリを勝手に開放されないようにstatic Matに格納
	gARTImageMat = matCV;
	// ARUint8にキャスト
	ARUint8 *imageBuffer = reinterpret_cast<ARUint8*>(gARTImageMat.data);

	return imageBuffer;
}

static void Idle(void)
{
	static int ms_prev;
	int ms;
	float s_elapsed;
	ARUint8 *image;

	ARMarkerInfo    *marker_info;					// Pointer to array holding the details of detected markers.
	int             marker_num;						// Count of number of markers detected.
	int             j, k;

	// Find out how long since Idle() last ran.
	ms = glutGet(GLUT_ELAPSED_TIME);
	s_elapsed = (float)(ms - ms_prev) * 0.001;
	if (s_elapsed < 0.01f) return; // Don't update more often than 100 Hz.
	ms_prev = ms;

	// Update drawing.
	DrawCubeUpdate(s_elapsed);

	// Grab a video frame.
	if ((image = getKinectImage()) != NULL) {
	//if ((image = arVideoGetImage()) != NULL) {
		gARTImage = image;	// Save the fetched image.
		gPatt_found = FALSE;	// Invalidate any previous detected markers.

		gCallCountMarkerDetect++; // Increment ARToolKit FPS counter.

		//// Detect the markers in the video frame.
		if (arDetectMarker(gARTImage, gARTThreshhold, &marker_info, &marker_num) < 0) {
			exit(-1);
		}

		//// Check through the marker_info array for highest confidence
		//// visible marker matching our preferred pattern.
		k = -1;
		for (j = 0; j < marker_num; j++) {
			if (marker_info[j].id == gPatt_id) {
				if (k == -1) k = j; // First marker detected.
				else if (marker_info[j].cf > marker_info[k].cf) k = j; // Higher confidence marker detected.
			}
		}

		if (k != -1) {
			// Get the transformation between the marker and the real camera into gPatt_trans.
			arGetTransMat(&(marker_info[k]), gPatt_centre, gPatt_width, gPatt_trans);
			gPatt_found = TRUE;
		}

		// Tell GLUT the display has changed.
		glutPostRedisplay();
	}
}

//
//	This function is called on events when the visibility of the
//	GLUT window changes (including when it first becomes visible).
//
static void Visibility(int visible)
{
	if (visible == GLUT_VISIBLE) {
		glutIdleFunc(Idle);
	}
	else {
		glutIdleFunc(NULL);
	}
}

//
//	This function is called when the
//	GLUT window is resized.
//
static void Reshape(int w, int h)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// Call through to anyone else who needs to know about window sizing here.
}

//
// This function is called when the window needs redrawing.
//
static void Display(void)
{
	GLdouble p[16];
	GLdouble m[16];

	// Select correct buffer for this context.
	glDrawBuffer(GL_BACK);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear the buffers for new frame.

	arglDispImage(gARTImage, &gARTCparam, 1.0, gArglSettings);	// zoom = 1.0.
	//arVideoCapNext();
	gARTImage = NULL; // Image data is no longer valid after calling arVideoCapNext().

	if (gPatt_found) {
		// Projection transformation.
		arglCameraFrustumRH(&gARTCparam, VIEW_DISTANCE_MIN, VIEW_DISTANCE_MAX, p);
		glMatrixMode(GL_PROJECTION);
		glLoadMatrixd(p);
		glMatrixMode(GL_MODELVIEW);

		// Viewing transformation.
		glLoadIdentity();
		// Lighting and geometry that moves with the camera should go here.
		// (I.e. must be specified before viewing transformations.)
		//none

		// ARToolKit supplied distance in millimetres, but I want OpenGL to work in my units.
		arglCameraViewRH(gPatt_trans, m, VIEW_SCALEFACTOR);
		glLoadMatrixd(m);

		// All other lighting and geometry goes here.
		DrawCube();
	} // gPatt_found

	// Any 2D overlays go here.
	//none

	glutSwapBuffers();
}


int main(int argc, char** argv)
{
	char glutGamemode[32];
	//const char *cparam_name = "Data\\camera_para.dat";
	const char *cparam_name_cv = "Data\\intrinsicParamKinect.xml";

	//
	// Camera configuration.
	//
#ifdef _WIN32
	char			*vconf = "Data\\WDM_camera_flipV.xml";
#else
	char			*vconf = "";
#endif
	const char *patt_name = "Data\\patt.sample1";

	// ----------------------------------------------------------------------------
	// Library inits.
	//

	glutInit(&argc, argv);

	// ----------------------------------------------------------------------------
	// Hardware setup.
	//

	if (!setupKinectV2ForARToolKit(cparam_name_cv, vconf, &gARTCparam)) {
	//if (!setupCamera(cparam_name, vconf, &gARTCparam)) {
		fprintf(stderr, "main(): Failed to set up AR camera for Kinect v2.\n");
		exit(-1);
	}
	prefWidth = kinectApp.widthColor;
	prefHeight = kinectApp.heightColor;

	if (!setupMarker(patt_name, &gPatt_id)) {
		fprintf(stderr, "main(): Unable to set up AR marker.\n");
		exit(-1);
	}

	// ----------------------------------------------------------------------------
	// Library setup.
	//

	// Set up GL context(s) for OpenGL to draw into.
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	if (!prefWindowed) {
		if (prefRefresh) sprintf_s(glutGamemode, "%ix%i:%i@%i", prefWidth, prefHeight, prefDepth, prefRefresh);
		else sprintf_s(glutGamemode, "%ix%i:%i", prefWidth, prefHeight, prefDepth);
		glutGameModeString(glutGamemode);
		glutEnterGameMode();
	}
	else {
		glutInitWindowSize(prefWidth, prefHeight);
		glutCreateWindow(argv[0]);
	}


	// Setup argl library for current context.
	if ((gArglSettings = arglSetupForCurrentContext()) == NULL) {
		fprintf(stderr, "main(): arglSetupForCurrentContext() returned error.\n");
		exit(-1);
	}
	debugReportMode();
	glEnable(GL_DEPTH_TEST);
	arUtilTimerReset();

	// Register GLUT event-handling callbacks.
	// NB: Idle() is registered by Visibility.
	glutDisplayFunc(Display);
	glutReshapeFunc(Reshape);
	glutVisibilityFunc(Visibility);
	glutKeyboardFunc(Keyboard);

	// Show help
	printf("\nInit succeeded.\n");
	printf("Press \"s\" for saving transformation matrix.\n");

	glutMainLoop();

	return (0);
}
