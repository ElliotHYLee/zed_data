#pragma once

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"


#include <iostream>
#include <ctype.h>
#include <algorithm> // for copy
#include <iterator> // for ostream_iterator
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>


using namespace std;
using namespace cv;


class Feature
{
private:
	int fast_threshold = 50;
	int fastFeatureTargetNumber;
	void setThreshold(int x);
public:
	Feature(int fastFeatureTargetNumber);
	~Feature();
	static void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2);
	static void featureDetection(cv::Mat img_1, vector<Point2f>& points1, int threshold);
	void adjustThreshold(vector<Point2f> points);
	int getThreshold();
};

