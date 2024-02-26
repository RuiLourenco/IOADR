#pragma once

#include "DisparityMap.h"
#include<Eigen/Core>
#include <vector>
#include "Image.h"

namespace lf{
	struct LfParameters {
		double f = 4.25;
		double dH = 25;
		double sensorSize = 35;
		double focalLength = 100;
	};
	class LightField {
	private:
		std::vector<std::vector<image::Image>> data;
		dmap::DisparityMap groundTruth;
		int size[5];
		LfParameters parameters;
		
		cv::Mat smoothMask;
		std::array<double, 2> dispRange;
		
		LfParameters parseParameterConfigFile(std::ifstream& parameterFile);
		std::string getNameFromPath(std::string);
		image::Image loadView(uint i,std::string path, const char* view_prefix);
		image::Image loadViewSF(char* n, char* m, std::string path, const char* view_prefix);
		void loadLightField(std::string path);
		void loadLightFieldStanford(std::string path);
		void parseDatasetString(std::string);

	public:
		bool isStanford = false;
		cv::Mat planeMask;
		std::string name;
		LightField(std::string path_to_folder, std::string dataset);
		
		const int viewHeight();
		const int viewWidth();
		const int viewMatrixHeight();
		const int viewMatrixWidth();
		const std::array<int, 2> getAngSize();
		const std::array<int, 2> getSpacialSize();
		const std::array<double, 2> getDispRange();
		const dmap::DisparityMap& getGroundTruth();
		
		const double getQm();
		const double getQn();
		const double getX(int m, double d);
		const double getY(int n, double d);
		const double getZ(double d);
		const double getDisparity(double z);
		cv::Mat disp2Depth(dmap::DisparityMap& disparityMap);
		const cv::Mat getXGrid(dmap::DisparityMap& disparityMap);
		const cv::Mat getYGrid(dmap::DisparityMap& disparityMap);


		const image::Image& getView(uint l,uint k);
		const image::Image getEpi(uint view, uint position, bool isHorizontal);
		const image::Image& getCentralView();
		const Eigen::Vector3d  getPoint(uint n, uint m, double d);
		const double getZFromPlaneAndCentralViewLocation(Eigen::Vector4d plane, int n, int m);

	};
}
