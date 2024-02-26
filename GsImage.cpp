#include "GsImage.h"
#include "Image.h"
#include <iostream>
#include <fstream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "MathUtils.h"

using namespace gsImage;
using namespace mu;

const double  GsImage::getValue(double n, double m) {

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
	uchar topLeftValue = (*this)(topLeft[0], topLeft[1]);
	uchar bottomLeftValue = (*this)(bottomLeft[0], bottomLeft[1]);
    uchar topRightValue = (*this)(topRight[0], topRight[1]);
	uchar bottomRightValue = (*this)(bottomRight[0], bottomRight[1]);

	uchar value  =(uchar) mu::bilinearInterpolation((double)topLeftValue, (double)topRightValue, (double)bottomLeftValue, (double)bottomRightValue, fractionalV, fractionalH);

	return value;
}

//cv::Vec3b Image::operator()(double n, double m) {
//	return getValue(n, m);
//}
GsImage::GsImage(const image::Image& colourImage) {
	cv::cvtColor(colourImage.data, this->data, cv::COLOR_BGR2GRAY);
}
GsImage::GsImage(unsigned int height, unsigned int width) {
	this->data = cv::Mat(height, width, CV_8U, cv::Scalar(0));
	this->title = "Image";
}
GsImage::GsImage(unsigned int height, unsigned int width, std::string title) {
	this->data = cv::Mat(height, width, CV_8U, cv::Scalar(0));
	this->title = title;
}

GsImage::GsImage(cv::Mat_<uchar> img) {

	this->data = img;
	this->title = "Image";
}
GsImage::GsImage(cv::Mat_<uchar> img, std::string title) {

	this->data = img;
	this->title = "Image";
}
GsImage::GsImage(std::string path) {
	size_t idx = path.find_last_of('\\');
	this->title = "Image";
	if (std::string::npos != idx)
	{
		this->title = path.substr(idx + 1);
	}
	this->data = cv::imread(path);
}

void const GsImage::show(const std::string& title) {


	std::string windowName = title; //Name of the window
	cv::namedWindow(windowName);
	cv::imshow(windowName, this->data);
	cv::waitKey(0);
}

void const GsImage::show() {
	show(this->title);
}

const void GsImage::saveAsImage(std::string imgPath) {
	cv::imwrite(imgPath, this->data); //
}


