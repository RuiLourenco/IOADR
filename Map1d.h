#pragma once
#pragma once
#include "Map.h"
#include <opencv2/core.hpp>



namespace mapd {
	class Map1d :public map::Map<double>
	{
	public:
		Map1d() = default;
		Map1d(uint height, uint width, std::string title);
		Map1d(cv::Mat data, std::string title);


		//DisparityMap(cv::Mat_<double> img);
		//DisparityMap(unsigned int n, unsigned int m, std::string);
		//DisparityMap(cv::Mat_<double> img, std::string);
		void show();
		void show(std::array<double, 2> range);
		void showDiff(Map1d map);
		double getValue(double n, double m);
		Map1d calcDiff(Map1d map);
		void saveAsImage(std::string path);
		void saveDiffAsImage(std::string path, Map1d map);
		void saveAsPFM(cv::String path);
		void saveAsCSV(cv::String path);
		void show(std::string);


	private:
		//
		std::string title;



	};
}
