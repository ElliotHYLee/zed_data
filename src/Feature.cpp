#include "Feature.h"

void Feature::setThreshold(int x)
{
	if (x <= 0) x = 0;
	this->fast_threshold = x;
}

Feature::Feature(int fastFeatureTargetNumber)
{
	this->fastFeatureTargetNumber = fastFeatureTargetNumber;
}

Feature::~Feature()
{}

void Feature::featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2) {

	//this function automatically gets rid of points for which tracking fails

	vector<float> err;
	Size winSize = Size(30, 30);
	TermCriteria termcrit = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01);

	vector<uchar> status;
	calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.01);

	//getting rid of points for which the KLT tracking failed or those who have gone outside the frame
	int indexCorrection = 0;
	for (int i = 0; i<status.size(); i++)
	{
		Point2f pt = points2.at(i - indexCorrection);
		if ((status.at(i) == 0) || (pt.x<0) || (pt.y<0)) {
			if ((pt.x<0) || (pt.y<0)) {
				status.at(i) = 0;
			}
			points1.erase(points1.begin() + (i - indexCorrection));
			points2.erase(points2.begin() + (i - indexCorrection));
			indexCorrection++;
		}
	}
}

void Feature::featureDetection(Mat img_1, vector<Point2f>& points1, int threshold) {   //uses FAST as of now, modify parameters as necessary
	vector<KeyPoint> keypoints_1;
	bool nonmaxSuppression = true;
	FAST(img_1, keypoints_1, threshold, nonmaxSuppression);
	KeyPoint::convert(keypoints_1, points1, vector<int>());
	//Point2f temp1(320, 180), temp2(960,180), temp3(320, 540), temp4(960, 540);
	//points1.push_back(temp1);
	//points1.push_back(temp2);
	//points1.push_back(temp3);
	//points1.push_back(temp4);
}

void Feature::adjustThreshold(vector<Point2f> points)
{
	int numFeat = points.size();
	if (numFeat < fastFeatureTargetNumber) this->setThreshold(fast_threshold - 1);
	else this->setThreshold(fast_threshold + 1);
}

int Feature::getThreshold()
{
	return this->fast_threshold;
}
