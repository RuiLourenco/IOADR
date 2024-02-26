#include "LightField.h"
#include "Image.h"
#include "MathUtils.h"
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/photo.hpp>
#include <filesystem>

using namespace lf;
using namespace dmap;

LfParameters LightField::parseParameterConfigFile(std::ifstream& parametersFile) {
	std::string auxString;
	LfParameters parameters;
	//File Cursor runs until it finds '=' and saves the rhs. Parameter order is important.
	if (parametersFile.is_open()) { // always check whether the file is open
		parametersFile >> auxString;
		parametersFile >> auxString;
		size_t pos = auxString.find("=");
		std::string token = auxString.substr(0, pos);
		auxString.erase(0, pos + 1);
		parameters.f = std::stod(auxString);
		parametersFile >> auxString;
		//cout << parametersString << endl;
		pos = auxString.find("=");
		token = auxString.substr(0, pos);
		auxString.erase(0, pos + 1);
		parameters.focalLength = std::stod(auxString);
		parametersFile >> auxString;
		pos = auxString.find("=");
		token = auxString.substr(0, pos);
		auxString.erase(0, pos + 1);
		parameters.dH = std::stod(auxString);
		parametersFile >> auxString;
		pos = auxString.find("=");
		token = auxString.substr(0, pos);
		auxString.erase(0, pos + 1);
		parameters.sensorSize = std::stod(auxString);
	}
	return parameters;
}

const double LightField::getX(int m, double d) {

	double x = m * getQm() * getZ(d);

	return x;
}
const double LightField::getQm() {
	double sensorSize = parameters.sensorSize;
	double Qm = 0.5 * (sensorSize / (size[3] - 1.)) * 1. / parameters.focalLength;
	return Qm;
}

const double LightField::getY(int n, double d) {

	double y = (n)*getQn() * getZ(d);

	return y;
}
const double LightField::getQn() {
	double f = parameters.focalLength;
	double sensorSize = parameters.sensorSize;
	double Qn = 0.5 * (sensorSize / (size[2] - 1.)) * 1. / parameters.focalLength;
	return Qn;
}

const Eigen::Vector3d  LightField::getPoint(uint n, uint m, double d) {
	return Eigen::Vector3d(getY(n, d), getX(m, d), getZ(d));
}
const double LightField::getZFromPlaneAndCentralViewLocation(Eigen::Vector4d plane, int n, int m) {
	double Qm = this->getQm();
	double Qn = this->getQn();

	double z = (-plane(3) / (plane(0) * Qn * n + plane(1) * Qm * m + plane(2)));
	return z;
}

const double LightField::getZ(double d) {
	double b = parameters.dH;
	double focalLength = parameters.focalLength;
	double f = parameters.f;
	double sensorSize = parameters.sensorSize;
	double q = b * focalLength * std::max(size[2], size[3]);
	double depth = 1.0 / ((1000.0 * sensorSize) * d / q + (1.0 / f));

	return depth;
}
const double LightField::getDisparity(double z) {
	double b = parameters.dH;
	double focalLength = parameters.focalLength;
	double f = parameters.f;
	double sensorSize = parameters.sensorSize;

	double ff = (b / 1000.) * focalLength * std::max(size[2], size[3]);
	double disp = (ff * f / z - ff) / f / sensorSize;

	return disp;
}
const cv::Mat LightField::getXGrid(DisparityMap& disparityMap) {
	double f = parameters.focalLength;
	double sensorSize = parameters.sensorSize;
	std::vector<int> t_x;
	for (int i = cv::Range(0, size[3] - 1).start; i <= cv::Range(0, size[3] - 1).end; i++) t_x.push_back(i);
	cv::Mat yy = cv::repeat(cv::Mat(t_x).t(), size[2],1);
	yy.convertTo(yy, CV_64F);
	yy = (0.5 * ((yy)*sensorSize / (size[2] - 1))).mul(this->disp2Depth(disparityMap)) / f;
	return yy;
}
const cv::Mat LightField::getYGrid(DisparityMap& disparityMap) {
	double f = parameters.focalLength;
	double sensorSize = parameters.sensorSize;
	std::vector<int> t_y;
	for (int i = cv::Range(0, size[2] - 1).start; i <= cv::Range(0, size[2] - 1).end; i++) t_y.push_back(i);
	cv::Mat yy = cv::repeat(cv::Mat(t_y), 1, size[3]);
	yy.convertTo(yy, CV_64F);
	yy = (0.5 * ((yy)*sensorSize / (size[2] - 1))).mul(this->disp2Depth(disparityMap)) / f;
	return yy;
}
cv::Mat LightField::disp2Depth(dmap::DisparityMap& disparityMap) {
	
	double b = parameters.dH;
	double focalLength = parameters.focalLength;
	double f = parameters.f;
	double sensorSize = parameters.sensorSize;
	double q = b * focalLength * std::max(size[2],size[3]);
	cv::Mat depth = 1.0 / ((1000.0 * sensorSize) * disparityMap.data / q + (1.0 / f));

	return depth;
}
std::string LightField::getNameFromPath(std::string path) {
	std::string name;
	//Trim trailing slash
	if (path.back() == '\\' || path.back() == '/') {
			path.pop_back();
	}
	//For Windows Style File Path
	size_t idx = path.find_last_of('\\');
	if (std::string::npos != idx)
	{
		name = path.substr(idx + 1);
	}
	else {
		//For Linux Style File Path
		size_t idx = path.find_last_of('/');
		if (std::string::npos != idx)
		{
			name = path.substr(idx + 1);
		}
	}
	return name;
}

image::Image LightField::loadView(uint i, std::string path,const char* view_prefix) {
	char imageNumber[4];
	snprintf(imageNumber, sizeof(imageNumber), "%03d", i);
	return image::Image(path + "\\" + view_prefix + imageNumber + ".png");
}
image::Image lf::LightField::loadViewSF(char* n, char* m, std::string path, const char* view_prefix)
{
	std::filesystem::path currentPath = path;
	std::string incompletePath = path + "\\" + view_prefix + n + "_" + m;
	std::cout << incompletePath << std::endl;
	
	try {
		auto files = std::filesystem::directory_iterator{ currentPath };
	}
	catch (const std::exception& ex) {
		std::cerr << ex.what() << std::endl;
		exit(-1);
	}
	for (auto& imagePath : std::filesystem::directory_iterator{ currentPath }) {

		std::string pathString = imagePath.path().string();
		if (pathString.find(incompletePath) != std::string::npos) {
			cv::Mat image = cv::imread(pathString);
			cv::resize(image, image, cv::Size(), 0.5, 0.5, cv::INTER_AREA);	
			image::Image view(image);
			return view;
		}
	}
}

void lf::LightField::loadLightField(std::string path)
{
	const char* view_prefix = "input_Cam";
	const int viewSize = 9;

	//Get Name from Path
	this->name = getNameFromPath(path);

	//Load LF Camera Parameters
	std::ifstream parametersFile;
	parametersFile.open(path + "\\parameters_.cfg");
	this->parameters = parseParameterConfigFile(parametersFile);
	parametersFile.close();
	//reserve light field vector
	std::vector<std::vector<image::Image>> dataInit(viewSize, std::vector<image::Image>(viewSize));
	this->data = dataInit;
	//load light field from disk
	for (uint i = 0; i < viewSize * viewSize; i++) {
		uint s = i % viewSize;
		uint t = i / viewSize;
		this->data[t][s] = loadView(i, path, view_prefix);
	}
	this->size[0] = viewSize;
	this->size[1] = viewSize;
	this->size[2] = this->data[0][0].rows();
	this->size[3] = this->data[0][0].cols();
	this->size[4] = this->data[0][0].data.channels();
	this->planeMask = cv::imread(path + "\\" + "mask_planes_lowres" + ".png", cv::IMREAD_ANYCOLOR);
	this->smoothMask = cv::imread(path + "\\" + "mask_smooth_surfaces_lowres" + ".png", cv::IMREAD_ANYCOLOR);
	cv::Mat gt = cv::imread(path + "\\" + "gt_disp_lowres.pfm", cv::IMREAD_UNCHANGED);
	gt.convertTo(gt, CV_64F);
	this->groundTruth = DisparityMap(this, gt.clone(), "Ground Truth");
	cv::minMaxLoc(gt, &this->dispRange[0], &dispRange[1]);
}

void lf::LightField::loadLightFieldStanford(std::string path)
{
	//this->size[0] = {9, 9, 512, 512, 3};
	const char* view_prefix = "out_";
	const int viewSize = 9;

	//camera parameters are set at 1 as they only have weight influence on the result;
	this->parameters.f = 1;
	this->parameters.focalLength = 1;
	this->parameters.dH = 1;
	this->parameters.sensorSize = 1;

	std::vector<std::vector<image::Image>> dataInit(viewSize, std::vector<image::Image>(viewSize));
	this->data = dataInit;
	uint l = 8;
	uint k = 0;
	for (uint i = 4; i < 13; i = i + 1, l--) {
		for (uint j = 4; j < 13; j = j + 1, k++) {
			char n[3];
			char m[3];
			snprintf(n, sizeof(n), "%02d", i);
			snprintf(m, sizeof(m), "%02d", j);
			this->data[l][k] = loadViewSF(n,m,path, view_prefix);
		}
		k = 0;
	}
	this->size[0] = viewSize;
	this->size[1] = viewSize;
	this->size[2] = this->data[0][0].rows();
	this->size[3] = this->data[0][0].cols();
	this->size[4] = this->data[0][0].data.channels();
	std::cout << this->size[0] <<" " <<this->size[1]<<" "<<this->size[2]<<" "<<this->size[3]<<std::endl;
	cv::Mat planeMask = cv::Mat(this->size[2], this->size[3], CV_8UC1, cv::Scalar(0));
	cv::Mat smoothMask = cv::Mat(this->size[2], this->size[3], CV_8UC1, cv::Scalar(0));
	this->planeMask = planeMask;
	this->smoothMask = smoothMask;
	cv::Mat gt = cv::Mat(this->size[2], this->size[3], CV_64FC1, cv::Scalar(0.0));
	this->groundTruth = DisparityMap(this, gt.clone(), "Ground Truth");
	this->dispRange[0] = -2;
	this->dispRange[1] = 2;
}
void lf::LightField::parseDatasetString(std::string datasetString)
{
	if (!datasetString.compare("HCI")) {
		this->isStanford = false;
	}
	else {
		if (!datasetString.compare("Stanford") || !datasetString.compare("SF")) {
			this->isStanford = true;
		}
	}
}
LightField::LightField(std::string path, std::string dataset) {
	

	parseDatasetString(dataset);
	if (this->isStanford) {
		loadLightFieldStanford(path);
	}
	else {
		loadLightField(path);
	}
}




const image::Image& LightField::getView(uint l, uint k) {
	return this->data[l][k];
}

const image::Image& LightField::getCentralView() {
	uint lr = viewMatrixHeight() / 2;
	uint kr = viewMatrixHeight() / 2;
	return getView(lr, kr);
}

const image::Image LightField::getEpi(uint view, uint position, bool isHorizontal) {

	int height;
	int width;
	if (isHorizontal) {
		height = viewMatrixWidth();
		width = viewWidth();
	}
	else {
		height = viewMatrixHeight();
		width = viewHeight();
	}
	image::Image EPI = image::Image(height, width, "EPI");
	for (uint shiftAngular = 0; shiftAngular < height; shiftAngular++) {

		image::Image currView;
		if (isHorizontal) {
			currView = this->getView(view, shiftAngular);
		}
		else {
			currView = this->getView(shiftAngular, view);
		}
		for (uint shiftSpacial = 0; shiftSpacial < width; shiftSpacial++) {
			if (isHorizontal) {
				
				EPI(shiftAngular, shiftSpacial) = currView(position, shiftSpacial);
			}
			else {
				EPI(shiftAngular, shiftSpacial) = currView(shiftSpacial, position);
			}
		}
	}
	return EPI;
}

const int LightField::viewHeight() {
	return size[2];
}
const int LightField::viewWidth(){ 
	return size[3];
}
const int LightField::viewMatrixHeight(){
	return size[0];
}
const int LightField::viewMatrixWidth(){
	return size[1];
}
const std::array<int, 2> LightField::getAngSize(){
	return { size[0],size[1] };
}
const std::array<int, 2> LightField::getSpacialSize(){
	return { size[2],size[3] };
}
const std::array<double, 2> LightField::getDispRange() {
	return this->dispRange;
}
const dmap::DisparityMap& LightField::getGroundTruth() {
	return this->groundTruth;
}
