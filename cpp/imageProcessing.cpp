/************************************************************************************************
 *contains the function that is used to identify the obstacles in the image and draw on the image
 ************************************************************************************************/
#ifdef unix
#include <iostream>


#endif
using namespace std;
using namespace cv;

IplImage* findObstacle(IplImage *in, int horizon, char color, vector<obstacle> &obstacles, char lighting)
{
	// creating ROI
	IplImage *out = cvCreateImage(cvGetSize(in), in->depth, in->nChannels);
	cvCopy(in, out, NULL);
	int x = out->origin;
	int y = 0;
	int width = out->width;
	int height = out->height + y;
	int add = 100;

	if (horizon < out->height)
	{
		y = out->origin + horizon;
	}
	else
	{
		y = out->origin;
	}

	CvRect water = cvRect(x, horizon, width, height);
	cvSetImageROI(in, water);
	// find low and high of the hue values
	IplImage *hsvImg = cvCreateImage(cvGetSize(in), IPL_DEPTH_8U, 3);
	IplImage *thresholded = cvCreateImage(cvGetSize(in), IPL_DEPTH_8U, 1);
	IplImage *thresholded2 = cvCreateImage(cvGetSize(in), IPL_DEPTH_8U, 1);
	IplImage *thresholded3 = cvCreateImage(cvGetSize(in), IPL_DEPTH_8U, 1);

	CvScalar hsv_min;
	CvScalar hsv_max;
	CvScalar hsv_min2;
	CvScalar hsv_max2;

	if (color == 'g')
	{
		hsv_min = cvScalar(50, 255, 255);
		hsv_max = cvScalar(70, 255, 255);
		hsv_min2 = cvScalar(30, 200, 50);
		hsv_max2 = cvScalar(60, 255, 255);
	}
	else if (color == 'r')
	{
		hsv_min = cvScalar(0, 150, 100);
		hsv_max = cvScalar(30, 255, 255);
		hsv_min2 = cvScalar(100, 150, 110);
		hsv_max2 = cvScalar(180, 255, 255);
	}
	else if (color == 'y')
	{
		hsv_min = cvScalar(0, 255, 255);
		hsv_max = cvScalar(0, 255, 255);
		hsv_min2 = cvScalar(75, 100, 80);
		hsv_max2 = cvScalar(95, 255, 255);
	}
	else if (color == 'b')
	{
		hsv_min = cvScalar(0, 255, 255);
		hsv_max = cvScalar(0, 255, 255);
		hsv_min2 = cvScalar(0, 60, 60);
		hsv_max2 = cvScalar(10, 255, 255);
	}
	else
	{
		hsv_min = cvScalar(0, 0, 0, 0);
		hsv_max = cvScalar(0, 0, 0, 0);
		hsv_min2 = cvScalar(0, 0, 0, 0);
		hsv_max2 = cvScalar(0, 0, 0, 0);
	}

	// convert image from RGB to HSV
	cvCvtColor(in, hsvImg, CV_RGB2HSV);

	// thresholding images
	cvInRangeS(hsvImg, hsv_min, hsv_max, thresholded);
	cvInRangeS(hsvImg, hsv_min2, hsv_max2, thresholded2);
	cvOr(thresholded, thresholded2, thresholded3);

	// find obstacles color to draw circle
	if (color != 'y')
	{
		cvSmooth(thresholded3, thresholded3, CV_GAUSSIAN, 3, 3);
	}

	cvErode(thresholded3, thresholded3, NULL, 2);
	cvDilate(thresholded3, thresholded3, NULL, 3);
	cvSmooth(thresholded3, thresholded3, CV_GAUSSIAN, 3, 3);

	// Use blob detection method
	CBlobResult blobs;
	blobs = CBlobResult(thresholded3, NULL, 0);

	obstacles.resize(blobs.GetNumBlobs());
	int k = 0;
	for (int i = 0; i < (blobs.GetNumBlobs()); i++)
	{
		//determine obstacle parameters
		float x = (float)(blobs.GetBlob(i)->MinX() + ((blobs.GetBlob(i)->MaxX() - blobs.GetBlob(i)->MinX()) / 2.0));
		float y = (float)(blobs.GetBlob(i)->MinY() + ((blobs.GetBlob(i)->MaxY() - blobs.GetBlob(i)->MinY()) / 2.0));
		float radius = (float)(blobs.GetBlob(i)->MaxY() - blobs.GetBlob(i)->MinY()) / 2;
		float diameter = 2 * radius;
		if (color == 'y' && radius > 5)	// get the bigger obstacles first
		{
			obstacles[k].x = x;
			obstacles[k].y = y;
			obstacles[k].radius = radius;
			k++;
		}
		else if (color == 'b' && radius > 20)
		{
			obstacles[k].x = x;
			obstacles[k].y = y;
			obstacles[k].radius = radius;
			k++;
		}
		else if (color != 'b' && color != 'y' && radius > 5)
		{
			obstacles[k].x = x;
			obstacles[k].y = y;
			obstacles[k].radius = radius;
			k++;
		}
	}

	// if the bigger obstacles found then those are the closest obstacles
	obstacles.resize(k);
	obstacle obstacleTemp;
	for (int j = 0; j < k - 1; j++)
	{
		for (int i = 0; i < k - 1; i++)
		{
			if (obstacles[i].y < obstacles[i + 1].y)
			{
				obstacleTemp = obstacles[i + 1];
				obstacles[i + 1] = obstacles[i];
				obstacles[i] = obstacleTemp;
			}
		}
	}

	// adjust the obstacle y location 
	for (int j = 0; j < k; j++)
	{
		obstacles[j].y = obstacles[j].y + horizon;
	}

	cvReleaseImage(&thresholded);
	cvReleaseImage(&thresholded2);
	cvReleaseImage(&hsvImg);
	cvReleaseImage(&out);
	return (thresholded3);
}

void drawObstacle(IplImage *out, vector<obstacle> &greenobstacles, vector<obstacle> &redobstacles, vector< obstacle > &yellowobstacles, vector< obstacle > &blueobstacles, vector< gate > &gates, vector< path > &paths, vector< wall > &greenWall, vector< wall > &redWall, vector< wall > &blueWall, CvPoint target)
{
	// draw the green obstacles
	for (unsigned int i = 0; i < greenobstacles.size(); i++)
	{
		CvPoint pt = cvPoint(cvRound(greenobstacles[i].x), cvRound(greenobstacles[i].y));
		cvCircle(out, pt, cvRound(greenobstacles[i].radius), CV_RGB(0, 128, 0), 3);
	}

	// draw the red obstacles
	for (unsigned int i = 0; i < redobstacles.size(); i++)
	{
		CvPoint pt = cvPoint(cvRound(redobstacles[i].x), cvRound(redobstacles[i].y));
		cvCircle(out, pt, cvRound(redobstacles[i].radius), CV_RGB(255, 0, 0), 3);
	}

	// draw the yellow obstacles
	for (unsigned int i = 0; i < yellowobstacles.size(); i++)
	{
		CvPoint pt = cvPoint(cvRound(yellowobstacles[i].x), cvRound(yellowobstacles[i].y));
		cvCircle(out, pt, cvRound(yellowobstacles[i].radius), CV_RGB(255, 255, 0), 3);
	}

	// draw the blue obstacles
	for (unsigned int i = 0; i < blueobstacles.size(); i++)
	{
		CvPoint pt = cvPoint(cvRound(blueobstacles[i].x), cvRound(blueobstacles[i].y));
		cvCircle(out, pt, cvRound(blueobstacles[i].radius), CV_RGB(0, 0, 255), 3);
	}

	// draw the gates
	for (unsigned int i = 0; i < gates.size(); i++)
	{
		cvLine(out, gates[i].green, gates[i].red, CV_RGB(255, 255, 255), 3);
	}

	// draw the path
	for (unsigned int i = 0; i < paths.size(); i++)
	{
		cvLine(out, paths[i].nearEnd, paths[i].farEnd, CV_RGB(0, 0, 0), 3);
	}

	cvLine(out, paths[0].nearEnd, target, CV_RGB(150, 0, 40), 3);

	for (unsigned int i = 0; i < greenWall.size(); i++)
	{
		cvLine(out, greenWall[i].nearEnd, greenWall[i].farEnd, CV_RGB(0, 128, 0), 3);
	}

	for (unsigned int i = 0; i < redWall.size(); i++)
	{
		cvLine(out, redWall[i].nearEnd, redWall[i].farEnd, CV_RGB(255, 0, 0), 3);
	}

	for (unsigned int i = 0; i < blueWall.size(); i++)
	{
		cvLine(out, blueWall[i].nearEnd, blueWall[i].farEnd, CV_RGB(0, 0, 255), 3);
	}
}

// read tuning parameters
void inputParams(float &closingOnGateDen, float &closingPWM, float &PWMoffset, float &maxThrottle, float &diffCoef, float &leftOff, float &rightOff)
{
	cout << "Closing Gate: ";
	cin >> closingOnGateDen;
	if (closingOnGateDen <= 1) closingOnGateDen = 1;
	else if (closingOnGateDen > 100) closingOnGateDen = 100;
	else closingOnGateDen = closingOnGateDen;

	cout << endl << "Closing PWM: ";
	cin >> closingPWM;
	if (closingPWM <= 0) closingPWM = 10;
	else if (closingPWM > 100) closingPWM = 100;
	else closingPWM = closingPWM;

	cout << endl << "Offset PWM : ";
	cin >> PWMoffset;
	if (PWMoffset<= 0) PWMoffset = 10;
	else if (PWMoffset > 100) PWMoffset = 100;
	else PWMoffset = PWMoffset;

	cout << endl << "Max Throttle PWM: ";
	cin >> maxThrottle;
	if (maxThrottle <= 0) maxThrottle = 10;
	else if (maxThrottle > 100) maxThrottle = 100;
	else maxThrottle = maxThrottle;

	cout << endl << "Differential Coefficient: ";
	cin >> diffCoef;
	if (diffCoef <= 0) diffCoef = 0;
	else if (diffCoef > 100) diffCoef = 100;
	else diffCoef = diffCoef;

	cout << endl << "Offset Left Drive Motor: ";
	cin >> leftOff;
	if (leftOff <= 0) leftOff = 0;
	else if (leftOff > 100) leftOff = 100;
	else leftOff = leftOff;

	cout << endl << "Offset Right Drive Motor: ";
	cin >> rightOff;
	if (leftOff <= 0) rightOff = 0;
	else if (leftOff > 100) rightOff = 100;
	else rightOff = rightOff;
}

// add a trackbar
void onTrackbarSlide(int pos)
{
	inv = pos;
}

// perform a canny edge detection on an image
IplImage* doCanny(IplImage *in, double lowThresh, double highThresh, double aperture)
{
	IplImage *out = cvCreateImage(cvGetSize(in), IPL_DEPTH_8U, 1);
	IplImage *grey = cvCreateImage(cvGetSize(in), IPL_DEPTH_8U, 1);
	if (in->nChannels != 1)
	{
		cout << "Image is not grey";
		cvCvtColor(in, grey, CV_RGB2GRAY);
		cvCanny(grey, out, lowThresh, highThresh, (int) aperture);
	}
	else
	{
		cvCanny(in, out, lowThresh, highThresh, (int) aperture);
	}

	cvReleaseImage(&grey);
	return (out);
}