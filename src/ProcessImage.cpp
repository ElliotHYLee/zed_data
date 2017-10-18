#include "ProcessImage.h"
#include "Feature.h"



Point2d ProcessImage::getIntPixel(Point2f pixel)
{
	Point2d result;
	result.x = (int)pixel.x;
	result.y = (int)pixel.y;
	if (result.x < 0) result.x = 0;
	if (result.x > ds.width) result.x = ds.width - 1;
	if (result.y < 0) result.y = 0;
	if (result.y > ds.height) result.y = ds.height - 1;
	return result;
}

void ProcessImage::getPrevWorldPoint(Point2f pixel, sl::float4 & world)
{
	Point2d p = getIntPixel(pixel);
	prevPointCloud.getValue(p.x, p.y, &world);
	world.y *= -1;
}

void ProcessImage::getCurrWorldPoint(Point2f pixel, sl::float4 & world)
{
	Point2d p = getIntPixel(pixel);
	pointCloud.getValue(p.x, p.y, &world);
	world.y *= -1;
}

void ProcessImage::getAvgWorldPoint(Point2f pixel, sl::float4 & world, int type)
{
	sl::float4 localWorld, totalWorld(0, 0, 0, 0);
	int cnt = 0;

	for (int i = -range / 2; i < range / 2; i++)
	{
		for (int j = -range / 2; j < range / 2; j++)
		{
			if (type==0) getPrevWorldPoint(pixel, localWorld);
			else getCurrWorldPoint(pixel, localWorld);
			if (!isnan(localWorld.x))
			{
				totalWorld += localWorld;
				cnt++;				
			}
		}
	}
	world = totalWorld / cnt;
}

void ProcessImage::onMouseCallBack(int32_t event, int32_t x, int32_t y, int32_t flag, void * param)
{
	if (event == CV_EVENT_LBUTTONDOWN) {
		Point2f temp;
		temp.x = x;
		temp.y = y;
		sl::float4 world;
		ProcessImage * theInstance = (ProcessImage*)param;
		theInstance->getAvgWorldPoint(temp, world, 1);
		cout.precision(2);
		cout << world.x << " " << world.y << " " << world.z << endl;
	}
}

void ProcessImage::saveToPrev()
{
	this->rgb.copyTo(prevRgb);
	this->gray.copyTo(prevGray);
	//this->depth.copyTo(prevDepth);
	//this->zDepth.copyTo(prevZDepth);
	this->pointCloud.copyTo(prevPointCloud);
}

void ProcessImage::getWorldPoints()
{
	prevWorld.clear();
	currWorld.clear();
	sl::float4 prevTemp, currTemp;
	Point2f prevPixelPoint, currPixelPoint;
	int prevX, prevY, currX, currY;
	if (prevPixel.size() < 300) return;
	else
	{
		int counter = 0;
		for (int i = 0; i < prevPixel.size(); i++)
		{
			if (counter >= 100) break;
			prevPixelPoint = prevPixel[i];
			getAvgWorldPoint(prevPixelPoint, prevTemp, 0);

			currPixelPoint = currPixel[i];
			getAvgWorldPoint(currPixelPoint, currTemp, 1);

			if (isinf(prevTemp.z) || isnan(prevTemp.z) || isinf(currTemp.z) || isnan(currTemp.z)
				|| prevTemp.z==0 || currTemp.z==0) continue;
			else
			{
				//prevWorld.push_back(prevTemp);
				//currWorld.push_back(currTemp);
				prev << prevTemp.x << " " << prevTemp.y << " " << prevTemp.z << " ";
				curr << currTemp.x << " " << currTemp.y << " " << currTemp.z << " ";
				counter++;
			}
		}
		count++;
		prev << endl;
		curr << endl;
		currPos << pos.x << " " << pos.y << " " << pos.z << " ";
		currPos << rot.x << " " << rot.y << " " << rot.z << endl;
	}
}

cv::Mat ProcessImage::privatePlotCircles(cv::Mat img, vector<Point2f> p, CvScalar color)
{
	cv::Mat result;
	img.copyTo(result);
	float meanX = 0, meanY = 0, x, y;
	for (int i = 0; i < p.size(); i++)
	{
		x = p[i].x;
		y = p[i].y;
		meanX = (meanX*(i)+x) / (i + 1.0);
		meanY = (meanY*(i)+y) / (i + 1.0);
		circle(result, Point(x, y), 1, color, 2, 8, 0);
	}
	return result;
}

cv::Mat ProcessImage::privatePlotCircles(cv::Mat img, vector<Point2f> p)
{
	CvScalar blue = CV_RGB(0, 0, 255, 0);
	return privatePlotCircles(img, p, blue);
}

ProcessImage::ProcessImage()
{
}

ProcessImage::ProcessImage(Size displaySize, sl::CalibrationParameters cp)
{
	//prev.open("prev.txt");
	//curr.open("curr.txt");
	currPos.open("pos.txt");
	this->ds = displaySize;
	this->cp = cp;
	std::cout << "Image Processsing Ready" << std::endl;
}

ProcessImage::~ProcessImage()
{
	//prev.close();
	//curr.close();
	currPos.close();
}

void ProcessImage::doYourWork()
{
	prevPixel.clear();
	currPixel.clear();
	if (isFirst) isFirst = false;
	else
	{
		Feature::featureDetection(prevGray, prevPixel, f.getThreshold());
		f.adjustThreshold(prevPixel);
		Feature::featureTracking(prevGray, gray, prevPixel, currPixel);
		getWorldPoints();

		//Mat temp;
		//temp = privatePlotCircles(prevRgb, prevPixel, blue);
		//temp = privatePlotCircles(temp, currPixel, red);
		
		//imshow("features", temp);
		//string name = to_string(count) + ".jpg";
		//imwrite(name, temp);

	}

	saveToPrev();
}

void ProcessImage::updateFrames(cv::Mat &rgb, cv::Mat &depth, sl::Mat &zedDepth, sl::Mat &pointCloud, double dt)
{
	cv::resize(rgb, this->rgb, ds);
	cv::cvtColor(this->rgb, gray, CV_BGR2GRAY);
	cv::resize(depth, this->depth, ds);
	
	zedDepth.copyTo(zDepth); // no resize
	pointCloud.copyTo(this->pointCloud); // no resize
	

//	cv::imshow("rgb", this->rgb);
//	cv::imshow("gray", this->gray);
	//cv::imshow("Depth", this->depth);	
}

void ProcessImage::updateFrames(cv::Mat & rgb, sl::Mat & pointCloud, double dt, sl::float3 transl, sl::float3  rot)
{
	this->dt = dt;
	cv::resize(rgb, this->rgb, ds);
	cv::cvtColor(this->rgb, gray, CV_BGR2GRAY);
	//pointCloud.copyTo(this->pointCloud); // no resize
	this->pos = transl;
	this->rot = rot;
	//cv::imshow("gray", this->gray);
	//cv::imshow("rgb", this->rgb);
	cv::imwrite("1.jpg", this->rgb);
}

//cv::Mat ProcessImage::getRGB()
//{
//	return rgb;
//}
//
//cv::Mat ProcessImage::getDepth()
//{
//	return depth;
//}
//
//cv::Mat ProcessImage::getGray()
//{
//	return gray;
//}
//

void ProcessImage::getInfo(cv::Mat left, cv::Mat right, sl::float3 rotation, sl::float3 translation, double freq)
{
	currPos << rotation.x << " " << rotation.y << " " << rotation.z << " " << translation.x << " " << translation.y << " " << translation.z << endl;

}