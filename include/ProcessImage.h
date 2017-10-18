#pragma once
#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>
#include <sl/Core.hpp>
#include <sl/defines.hpp>
#include "Feature.h"


using namespace cv;
using namespace std;



class ProcessImage
{


private:
	ofstream prev, curr, currPos;
	int count=0;

	CvScalar blue = CV_RGB(0, 0, 255, 0);
	CvScalar red = CV_RGB(255, 0, 0, 0);

	sl::CalibrationParameters cp;
	Size ds;
	bool isFirst = true;
	Feature f = Feature(1000*1);

	vector<Point2f> prevPixel, currPixel;
	vector<sl::float4> prevWorld, currWorld;
	sl::float3 pos, rot;
	double dt;
	int range = 10;

	// t-1 images
	cv::Mat prevRgb, prevDepth, prevGray;
	sl::Mat prevZDepth, prevPointCloud;

	// current images
	cv::Mat rgb, depth, gray;
	sl::Mat zDepth, pointCloud;

	// functions
	Point2d getIntPixel(Point2f pixel);
	
	void saveToPrev();
	void getWorldPoints();

	cv::Mat privatePlotCircles(cv::Mat img, vector<Point2f> p);
	cv::Mat privatePlotCircles(cv::Mat img, vector<Point2f> p, CvScalar blue);

	// world coordinate function

public:
	ProcessImage();
	ProcessImage(Size displaySize, sl::CalibrationParameters cp);
	~ProcessImage();
	
	void doYourWork();

	void updateFrames(cv::Mat &rgb, cv::Mat &depth, sl::Mat & zedDepth, sl::Mat& pointCloud, double dt);
	void updateFrames(cv::Mat &rgb, sl::Mat& pointCloud, double dt, sl::float3 transl, sl::float3  rot);
	
	void getPrevWorldPoint(Point2f pixel, sl::float4 &world);
	void getCurrWorldPoint(Point2f pixel, sl::float4 &world);
	void getAvgWorldPoint(Point2f pixel, sl::float4 &world, int type);
	////void showCircles();
	//cv::Mat getRGB();
	//cv::Mat getDepth();
	//cv::Mat getGray();

	void getInfo(cv::Mat left, cv::Mat right, sl::float3 rotation, sl::float3 translation, double freq);


	static void onMouseCallBack(int32_t event, int32_t x, int32_t y, int32_t flag, void * param);
};

