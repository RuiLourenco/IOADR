#include "Image.h"
#include <iostream>
#include <fstream>
#include <opencv2/highgui.hpp>
#include "MathUtils.h"

using namespace image;
using namespace mu;

const cv::Vec3d  Image::getValue(double n, double m) {

	double fractionalH = m - std::floor(m);
	double fractionalV = n - std::floor(n);

	if (fractionalH < 0.01 && fractionalV < 0.01) {
		return ((*this)((uint)n, (uint)m));
	}
	
	std::array<uint, 2> topLeft;
	std::array<uint, 2> topRight;
	std::array<uint, 2> bottomLeft;
	std::array<uint, 2> bottomRight;
	mu::getSurroundingPixels(topLeft, topRight, bottomLeft, bottomRight, n, m);
	//This keeps only the fractional part. Only works because n,m > 0!
	
	//Gets the values;
	cv::Vec3b topLeftValue = (*this)(topLeft[0], topLeft[1]);
	cv::Vec3b bottomLeftValue = (*this)(bottomLeft[0], bottomLeft[1]);
	cv::Vec3b topRightValue = (*this)(topRight[0], topRight[1]);
	cv::Vec3b bottomRightValue = (*this)(bottomRight[0], bottomRight[1]);

	cv::Vec3d value = { 0.,0.,0. };
	//for each color channel
	for (int c = 0; c < 3; c++) {
		value[c] = mu::bilinearInterpolation((double)topLeftValue[c], (double)topRightValue[c], (double)bottomLeftValue[c], (double)bottomRightValue[c], fractionalV, fractionalH);
	}
	return value;
}

//cv::Vec3b Image::operator()(double n, double m) {
//	return getValue(n, m);
//}

Image::Image(unsigned int height, unsigned int width) {
	this->data = cv::Mat(height, width, CV_8UC3, cv::Scalar(0,0,0));
	this->title = "Image";
}
Image::Image(unsigned int height, unsigned int width, std::string title) {
	
	this->data = cv::Mat(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
	this->title = title;
}

Image::Image(cv::Mat_<cv::Vec3b> img) {
	
	this->data = img;
	this->title = "Image";
}
Image::Image(cv::Mat_<cv::Vec3b> img,std::string title) {

	this->data = img;
	this->title = "Image";
}
Image::Image(std::string path) {
	size_t idx = path.find_last_of('\\');
	this->title = "Image";
	if (std::string::npos != idx)
	{
		this->title = path.substr(idx + 1);
	}
	this->data = cv::imread(path);
}

void const Image::show(const std::string& title) {


	std::string windowName = title; //Name of the window
	cv::namedWindow(windowName);
	cv::imshow(windowName, this->data);
	cv::waitKey(0);
}

 void const Image::show() {
	show(this->title);
}


const void Image::showDiff(Image map) {
	calcDiff(map).show();
}

Image Image::calcDiff(Image map) {
	cv::Mat dst;
	cv::absdiff(this->data, map.data,dst);
	Image diffImage = Image(dst, "difference");
	return diffImage;
}

const void Image::saveAsImage(std::string imgPath) {
	cv::imwrite(imgPath, this->data); //
}
const void Image::saveDiffAsImage(cv::String path, Image map) {
	calcDiff(map).saveAsImage(path);
}

