#pragma once
#include "Map.h"
#include "Map1d.h"
//#include "NormalMaps.h"
#include <opencv2/core.hpp>
#include "Parameters.h"

namespace lf {
	class LightField;
}
namespace nm {
	class NormalMap;
}


namespace dmap{

	struct Results {
		double mse = 0;
		double badpix = 0;
		double maePlanes = 0;

		static const std::string csvHeader() {
			std::ostringstream csvHeader1;
			csvHeader1 << "mse, badpix, maePlanes";
			return csvHeader1.str();
		}
		const std::string toCSV() {
			std::ostringstream csvLine;
			
			csvLine	<< mse << ","
				<< badpix << ","
				<< maePlanes;
			return csvLine.str();
		}
	};

	class DisparityMap :public map::Map<double>
	{
	public:
		lf::LightField* lightField;

		DisparityMap() = default;
		DisparityMap(lf::LightField* lightField, std::string title);
		DisparityMap(lf::LightField* lightField, cv::Mat data, std::string title);
		static DisparityMap fromStructureTensor(lf::LightField&,std::string title,double innerScale, double outerScale);
		static DisparityMap fromIOADR(lf::LightField& lightField, idi::Parameters parameters);
		static DisparityMap fromIOADR(lf::LightField& lightField, idi::Parameters parameters,idi::Settings settings);
		Results getResults();
		double badPix(double threshold);
		double maePlanes();
		
		//DisparityMap(cv::Mat_<double> img);
		//DisparityMap(unsigned int n, unsigned int m, std::string);
		//DisparityMap(cv::Mat_<double> img, std::string);
		void show();
		void show(std::array<double, 2> range);
		void showDiff(DisparityMap map);
		double getValue(double n, double m);
		mapd::Map1d calcDiff(DisparityMap map);
		void saveAsImage(std::string path);
		void saveDiffAsImage(std::string path, DisparityMap map);
		void saveDiffAsImage(cv::String path);
		void saveAsPFM(cv::String path);
		void saveAsCSV(cv::String path);
		double mse(int borderOffset = 0);
		const double inRange(double d);
		nm::NormalMap estimateNormals();

	private:
		//
		std::string title;
		


		void show(std::string);


	};
}
