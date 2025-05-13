#include "DisparityMap.h"
#include <iostream>
#include <fstream>
#include <opencv2/highgui.hpp>
#include "StructureTensorEstimator.h"
#include "LightField.h"
#include "MathUtils.h"
#include "NormalMaps.h"
#include "IterativeDisparityImprovement.h"

using namespace dmap;



DisparityMap::DisparityMap(lf::LightField* lightField, cv::Mat data, std::string title)
{

	this->lightField = lightField;
	this->data = data;
	this->title = title;
}
DisparityMap::DisparityMap(lf::LightField* lightField, std::string title) {
	int height = lightField->viewHeight();
	int width = lightField->viewWidth();
	this->lightField = lightField;
	this->data = cv::Mat(height, width, CV_64F, cv::Scalar(0.0));
	this->title = title;
}

DisparityMap DisparityMap::fromStructureTensor(lf::LightField& lightField, std::string title, double innerScale, double outerScale) {
	ste::StructureTensorEstimator stest(lightField, innerScale, outerScale);
	dmap::DisparityMap dMap = stest.estimateDisparity();
	
	return dMap;
}
DisparityMap DisparityMap::fromIOADR(lf::LightField& lightField, idi::Parameters parameters) {
	std::cout << "GROUND TRUTH LIGHT FIELD REFERENCE TEST 2: "<< lightField.getGroundTruth().lightField->name << std::endl;
	idi::IterativeDisparityImprovement depthEstimator(lightField, parameters);
	std::cout << "GROUND TRUTH LIGHT FIELD REFERENCE TEST 3: " << lightField.getGroundTruth().lightField->name << std::endl;
	DisparityMap dMap = depthEstimator.run();
	return dMap;
}
DisparityMap DisparityMap::fromIOADR(lf::LightField& lightField, idi::Parameters parameters,idi::Settings settings) {
	idi::IterativeDisparityImprovement depthEstimator(lightField, parameters,settings);
	DisparityMap dMap = depthEstimator.run();
	return dMap;
}
double DisparityMap::getValue(double n, double m) {
	std::array<uint, 2> topLeft;
	std::array<uint, 2> topRight;
	std::array<uint, 2> bottomLeft;
	std::array<uint, 2> bottomRight;
	double fractionalH = m - floor(m);
	double fractionalV = n - floor(n);
	mu::getSurroundingPixels(topLeft, topRight, bottomLeft, bottomRight, n, m);
	
	if (fractionalH < 0.01 && fractionalV < 0.01) {
		return ((*this)((uint)n, (uint)m));
	}
	if (fractionalH > 0.99 && fractionalV > 0.99) {
		return ((*this)((uint)n+1, (uint)m+1));
	}

	if (fractionalV < 0.01 || fractionalV > 0.99) {
		std::array<uint, 2> left = { round(n),topLeft[1]};
		std::array<uint, 2> right = { round(n),topRight[1]};
		//std::cout << "(" << n << "," << m << ")" << "(" << left[0] << "," << left[1] << ")" << std::endl;
		double leftValue = (*this)(left[0], left[1]);
		double rightValue = (*this)(right[0], right[1]);
		return mu::lerp(leftValue, rightValue, fractionalH);
	}
	if (fractionalH < 0.01 || fractionalH> 0.99) {
		std::array<uint, 2> top = { topLeft[0],std::round(m)};
		std::array<uint, 2> bottom = { bottomLeft[0],std::round(m)};
		double topValue = (*this)(top[0], top[1]);
		double bottomValue = (*this)(bottom[0], bottom[1]);
		//std::cout << topValue << "," << bottomValue << "," << fractionalV << std::endl;
		return mu::lerp(topValue, bottomValue, fractionalV);
	}
	
	double topLeftValue = (*this)(topLeft[0], topLeft[1]);
	double bottomLeftValue = (*this)(bottomLeft[0], bottomLeft[1]);
	double topRightValue = (*this)(topRight[0], topRight[1]);
	double bottomRightValue = (*this)(bottomRight[0], bottomRight[1]);

	//std::cout << "fractionalV = " << fractionalV << " fractionalU = " << fractionalH << std::endl;
	//std::cout << "topLeft = " << topLeftValue << " topRight = " << topRightValue << " bottomLeft = " << bottomLeftValue << " bottomRight = " << bottomRightValue << std::endl;
	double valueU = mu::lerp(topLeftValue, topRightValue, fractionalH);
	double valueD = mu::lerp(bottomLeftValue, bottomRightValue, fractionalH);
	double valueTest = mu::lerp(valueU, valueD, fractionalV);
	double value = mu::bilinearInterpolation(topLeftValue, topRightValue, bottomLeftValue, bottomRightValue, fractionalV, fractionalH);
	//std::cout << " valueU = " << valueU << " valueD = " << valueD << " valueTest = " << valueTest << "value = " << value << std::endl;
	return value;
}


void DisparityMap::show(std::string title) {
	cv::Mat viz = this->data.clone();
	double minVal, maxVal;
	minMaxLoc(viz, &minVal, &maxVal);
	cv::normalize(viz, viz, 0, 255, cv::NORM_MINMAX, CV_8UC1);
	std::string windowName = title; //Name of the window
	cv::namedWindow(windowName);
	cv::imshow(windowName, viz);
	cv::waitKey(0);
}

void DisparityMap::show() {
	show(this->title);
}

void DisparityMap::show(std::array<double,2> range) {
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


void DisparityMap::showDiff(DisparityMap map) {
	std::array<double, 2> range = { 0,0.2 };
	calcDiff(map).show(range);
}

mapd::Map1d DisparityMap::calcDiff(DisparityMap map) {
	cv::Mat dst;
	cv::absdiff(this->data, map.data, dst);
	mapd::Map1d diffDisparityMap(dst, "difference");
	return diffDisparityMap;
}

void DisparityMap::saveAsImage(std::string imgPath) {
	cv::Mat viz = this->data.clone();
	double minVal, maxVal;
	minMaxLoc(viz, &minVal, &maxVal);
	cv::normalize(viz, viz, 0, 255, cv::NORM_MINMAX, CV_8UC1);
	cv::imwrite(imgPath, viz); //
}
void DisparityMap::saveDiffAsImage(cv::String path, DisparityMap map) {
	cv::Mat diff = calcDiff(map);
	cv::Mat viz = diff;
	std::array<double, 2> range = { 0,0.2 };
	for (int n = 0; n < rows(); n++) {
		for (int m = 0; m < cols(); m++) {
			*viz.ptr<double>(n, m) = std::clamp(*viz.ptr<double>(n, m), range[0], range[1]);
		}
	}
	double minVal, maxVal;
	minMaxLoc(viz, &minVal, &maxVal);
	cv::normalize(viz, viz, 0, 255, cv::NORM_MINMAX, CV_8UC1);
	cv::imwrite(path, viz);
}
void DisparityMap::saveDiffAsImage(cv::String path) {
	DisparityMap groundTruth = lightField->getGroundTruth();
	saveDiffAsImage(path,groundTruth);
}
const double DisparityMap::inRange(double d) {
	return std::clamp(d, lightField->getDispRange()[0], lightField->getDispRange()[1]);
}
nm::NormalMap DisparityMap::estimateNormals() {
	nm::NormalMap normal = nm::NormalMap::estimateNormals(*this);
	return normal;
}

double DisparityMap::badPix(double threshold) {
	if (this->lightField->isStanford) {
		return 0;
	}
	//double borderSize = this->settings.basicSettings.borderSize;
	DisparityMap groundTruth = this->lightField->getGroundTruth();
	int borderSize = 15;
	cv::Rect roi = cv::Rect(borderSize, borderSize, this->lightField->viewWidth() - borderSize * 2, this->lightField->viewHeight() - borderSize * 2);
	cv::Mat gtRoi(groundTruth.data, roi);
	cv::Mat dmRoi(this->data, roi);
	cv::Mat tmp;
	cv::absdiff(dmRoi, gtRoi, tmp);
	int total = dmRoi.total();
	int accum = 0;
	for (int i = 0; i < (int)this->lightField->viewHeight() - 2 * borderSize; i++) {
		for (int j = 0; j < (int)this->lightField->viewWidth() - 2 * borderSize; j++) {
			double gtValue = gtRoi.at<double>(i, j);
			double dispValue = dmRoi.at<double>(i, j);
			double diff = tmp.at<double>(i, j);
			if (diff > threshold) {
				accum++;
			}
		}
	}

	return accum / (double)total;
}
double DisparityMap::maePlanes() {
	if (this->lightField->isStanford) {
		return 0;
	}
	DisparityMap groundTruth = this->lightField->getGroundTruth();
	nm::NormalMap gtNormalMap = groundTruth.estimateNormals();
	cv::Mat mask = lightField->planeMask;
	if (mask.empty()) {
		return -1;
	}
	nm::NormalMap normal = this->estimateNormals();
	double maePlanes = normal.medianAngleError(gtNormalMap, mask);
	return maePlanes;
}
Results DisparityMap::getResults() {
	if (this->lightField->isStanford) {
		return Results{ 0,0,0 };
	}
	return Results{ mse(15),badPix(0.07),maePlanes() };
}
double DisparityMap::mse(int borderOffset) {
	if (this->lightField->isStanford) {
		return 0;
	}
	double borderSize = borderOffset;
	DisparityMap groundTruth = this->lightField->getGroundTruth();
	cv::Rect roi = cv::Rect(borderSize, borderSize, this->cols() - borderSize * 2, this->rows() - borderSize * 2);
	cv::Mat gtRoi(groundTruth.data, roi);
	cv::Mat dmRoi(this->data, roi);
	cv::Mat tmp;
	absdiff(dmRoi, gtRoi, tmp);
	tmp = tmp.mul(tmp);
	cv::Scalar s = sum(tmp);
	double sse = s.val[0];
	double mse = sse / dmRoi.total();
	return mse * 100;
}

