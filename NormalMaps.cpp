#include "NormalMaps.h"
#include "Image.h"
#include "MathUtils.h"
#include "DisparityMap.h"
#include "LightField.h"
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>

using namespace nm;

Normal::Normal(Eigen::Vector3d normal) {
	/*if (normal(2) < 0) {
		normal = -1 * normal;
	}*/
	this->normal = normal.normalized();
}
Normal Normal::fromTangents(Eigen::Vector3d tangentH, Eigen::Vector3d tangentV) {
	Eigen::Vector3d normal = tangentH.cross(tangentV);
	normal = normal.normalized();
	/*if (normal(2) < 0) {
		normal = -1*normal;
	}*/
	return normal;
}
NormalMap  NormalMap::estimateNormals(dmap::DisparityMap disparityMap) {

	lf::LightField lightField = *disparityMap.lightField;

	cv::Mat xx = lightField.getXGrid(disparityMap);
	cv::Mat yy = lightField.getYGrid(disparityMap);
	cv::Mat zz = lightField.disp2Depth(disparityMap);

	cv::Mat kernel = (cv::Mat_<float>(3, 3) << 3, 0, -3, 10, 0, -10, 3, 0, -3);
	kernel /= 64.;
	//Approximate dM and dN vectors
	cv::Mat dxdx;
	filter2D(xx, dxdx, -1, kernel);
	cv::Mat dydx;
	filter2D(yy, dydx, -1, kernel);
	cv::Mat dzdx;
	filter2D(zz, dzdx, -1, kernel);
	cv::Mat dxdy;
	filter2D(xx, dxdy, -1, kernel.t());
	cv::Mat dydy;
	filter2D(yy, dydy, -1, kernel.t());
	cv::Mat dzdy;
	filter2D(zz, dzdy, -1, kernel.t());
	//cross product;
	cv::Mat Red = -(dzdx.mul(dxdy) - dxdx.mul(dzdy));
	cv::Mat Green = -(dydx.mul(dzdy) - dzdx.mul(dydy));
	cv::Mat Blue = -(dxdx.mul(dydy) - dydx.mul(dxdy));

	cv::Mat SquaredMagnitude = Red.mul(Red) + Green.mul(Green) + Blue.mul(Blue);
	cv::Mat Magnitude;
	cv::sqrt(SquaredMagnitude, Magnitude);
	Red = Red / Magnitude;
	Blue = Blue / Magnitude;
	Green = Green / Magnitude;

	cv::Mat vec[3] = { Red,Green,Blue };
	cv::Mat normals;
	cv::merge(vec, 3, normals);
	return NormalMap(normals);


}
const Eigen::Vector3d Normal::getNormal() {
	return normal;
}
Normal::operator Eigen::Vector3d() const {
	return normal;
}
double NormalMap::medianAngleError(NormalMap map2, cv::Mat mask) {
	std::vector<double> vec = angleErrorVec(map2, mask);
	double mae = mu::median(vec);
	return mae;
}
std::vector<double> NormalMap::angleErrorVec(NormalMap normalMap2, cv::Mat mask) {
	cv::Mat angularError = calcAngleError(normalMap2);
	std::vector<double> vec;
	double mae = 0;
	for (int n = 0; n < angularError.rows; n++) {
		for (int m = 0; m < angularError.cols; m++) {
			if (mask.at<uchar>(n, m) != 0) {
				double angularErrorScalar = angularError.at<double>(n, m);
				vec.push_back(angularErrorScalar);
			}
		}
	}
	return vec;
}
cv::Mat NormalMap::calcAngleError(NormalMap map2) {
	cv::Mat src;
	cv::Mat map2Data = map2;
	try {
		src = this->data.mul(map2Data);
	}
	catch (cv::Exception& e) {
		std::cerr << e.what() << std::endl;
	}

	cv::Mat tmp;

	cv::reduce(src.reshape(1, src.rows * src.cols), tmp, 1, 0);

	tmp = tmp.reshape(0, rows());
	cv::Mat numericMask;
	cv::Mat angularError;
	inRange(tmp, cv::Scalar(-1.0), cv::Scalar(1.0), numericMask);
	tmp.copyTo(angularError, numericMask);

	double sum_before = 0.;
	double sum_after = 0.;
	int count = 0;
	std::for_each(angularError.begin<double>(), angularError.end<double>(), [](double& e) { e = 180 / 3.1415 * acos(e); });
	return angularError;
}
NormalMap::NormalMap(uint height, uint width)  {
	this->data = cv::Mat(height, width, CV_64FC3, cv::Scalar(0, 0, -1));
}
NormalMap::NormalMap(cv::Mat_<cv::Vec3d> image) {
	this->data = image;
}



cv::Mat NormalMap::getIntegerNormalizedNormalMap(bool reverseColorChannels) {

	cv::Mat integerNormals;
	integerNormals = (this->data + 1) * 255 / 2;
	std::cout << integerNormals.at<cv::Vec3d>(30, 30) << std::endl;
	integerNormals.convertTo(integerNormals, CV_8UC3);
	if (reverseColorChannels) {
		cv::cvtColor(integerNormals, integerNormals, cv::COLOR_RGB2BGR);
	}
	return integerNormals;
}
void const NormalMap::show(){
	cv::Mat colorNormals = getIntegerNormalizedNormalMap(true);
	cv::namedWindow("CV2W Normal Map");
	cv::setWindowProperty("CV2W Normal Map", cv::WND_PROP_TOPMOST, cv::WINDOW_NORMAL);
	cv::imshow("CV2W Normal Map", colorNormals);
	//std::cout<<colorNormals.at<cv::Vec3b>(44,44)
	cv::waitKey(0);
}
void NormalMap::save(std::string path) {
	cv::Mat colorNormals = getIntegerNormalizedNormalMap(true);
	imwrite(path, colorNormals);
}

void NormalMap::setNormal(Normal& normal, uint n, uint m) {
	Eigen::Vector3d eNormal = normal;
	cv::Vec3d& normalVec = *this->data.ptr<cv::Vec3d>(n, m);
	cv::eigen2cv(eNormal, normalVec);
}
Normal NormalMap::getNormal(uint n, uint m) {
	cv::Vec3d normalVec = *this->data.ptr<cv::Vec3d>(n, m);
	//std::cerr << planeVec << std::endl;
	Eigen::Vector3d normal(normalVec[0], normalVec[1], normalVec[2]);
	//std::cerr << normal.transpose() << std::endl;
	return normal;
}
