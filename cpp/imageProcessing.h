#ifndef IMAGEPROCESSING_H
#define IMAGEPROCESSING_H

using namespace std;
using namespace cv;

IplImage* findObstacle(IplImage* in, int horizon, char color, vector<obstacle> &obstacles, char lighting);
void inputParams(float &closingOnGateDen, float &closingPWM, float &PWMoffset, float &maxThrottle, float &diffCoef, float &leftOff, float &rightOff);
void drawObstacle(IplImage* out, vector<obstacle> &greenobstacles, vector<obstacle> &redobstacles, vector<obstacle> &yellowobstacles, vector<obstacle> &blueobstacles, vector<gate> &gates, vector<path> &path,	vector<wall> &greenWall, vector<wall> &redWall, vector<wall> &blueWall, CvPoint target);
void onTrackbarSlide(int pos);
IplImage* doCanny(IplImage* in, double lowThresh, double highThresh, double aperture);

#endif