#include "Map1d.h"
#include <iostream>
#include <fstream>
#include <opencv2/highgui.hpp>

#include "MathUtils.h"
using namespace mapd;



Map1d::Map1d(cv::Mat data, std::string title):title(title)
{
	this->data = data;
}
Map1d::Map1d(uint height,uint width, std::string title):title(title){
	this->data = cv::Mat(height, width, CV_64F, cv::Scalar(0.0));
	this->title = title;
}

double Map1d::getValue(double n, double m) {
	std::array<uint, 2> topLeft;
	std::array<uint, 2> topRight;
	std::array<uint, 2> bottomLeft;
	std::array<uint, 2> bottomRight;
	mu::getSurroundingPixels(topLeft, topRight, bottomLeft, bottomRight, n, m);

	double fractionalH = m - floor(m);
	double fractionalV = n - floor(n);

	double topLeftValue = (*this)(topLeft[0], topLeft[1]);
	double bottomLeftValue = (*this)(bottomLeft[0], bottomLeft[1]);
	double topRightValue = (*this)(topRight[0], topRight[1]);
	double bottomRightValue = (*this)(bottomRight[0], bottomRight[1]);

	double value = mu::bilinearInterpolation(topLeftValue, topRightValue, bottomLeftValue, bottomRightValue, fractionalV, fractionalH);
	return value;
}


void Map1d::show(std::string title) {
	//clone data into temporary visualisation matrix
	cv::Mat viz = this->data.clone();
	double minVal, maxVal;
	minMaxLoc(viz, &minVal, &maxVal);
	std::cout << minVal << " " << maxVal << std::endl;
	//Normalize data between 0 and 255 and convert to uchar matrix.
	cv::normalize(viz, viz, 0, 255, cv::NORM_MINMAX, CV_8UC1);
	//Name the window
	std::string windowName = title;
	cv::namedWindow(windowName);
	//show
	cv::imshow(windowName, viz);
	//wait for click before closing the window
	cv::waitKey(0);
}
void Map1d::show(std::array<double, 2> range) {
	//clone data into temporary visualisation matrix
	cv::Mat viz = this->data.clone();
	for (int n = 0; n < rows(); n++) {
		for (int m = 0; m < cols(); m++) {
			*viz.ptr<double>(n, m) = std::clamp(*viz.ptr<double>(n, m), range[0], range[1]);
		}
	}
	double minVal, maxVal;
	minMaxLoc(viz, &minVal, &maxVal);
	std::cout << minVal << " " << maxVal << std::endl;
	//Normalize data between 0 and 255 and convert to uchar matrix.
	cv::normalize(viz, viz, 0, 255, cv::NORM_MINMAX, CV_8UC1);
	//Name the window
	std::string windowName = title;
	cv::namedWindow(windowName);
	//show
	cv::imshow(windowName, viz);
	//wait for click before closing the window
	cv::waitKey(0);
}

void Map1d::show() {
	show(this->title);
}


void Map1d::showDiff(Map1d map) {
	calcDiff(map).show();
}

Map1d Map1d::calcDiff(Map1d map) {
	cv::Mat dst;
	cv::absdiff(this->data, map.data, dst);
	Map1d diffDisparityMap = Map1d(dst, "difference");
	return diffDisparityMap;
}

void Map1d::saveAsImage(std::string imgPath) {
	cv::imwrite(imgPath, this->data); //
}
void Map1d::saveDiffAsImage(cv::String path, Map1d map) {
	calcDiff(map).saveAsImage(path);
}

