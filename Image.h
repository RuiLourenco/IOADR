#pragma once
#include "Map.h"
#include <opencv2/core.hpp>

namespace image {
	class Image :public map::Map<cv::Vec3b>
	{
	public:
		Image() = default;
		Image(unsigned int height, unsigned int width);
		Image(cv::Mat_<cv::Vec3b> img);
		Image(unsigned int height, unsigned int width, std::string);
		Image(cv::Mat_<cv::Vec3b> img, std::string);
		Image(std::string);

		const cv::Vec3d getValue(double n, double m);
		//cv::Vec3b operator()(double n, double m);

		void const show();
		const void showDiff(Image map);
		Image calcDiff(Image map);
		const void saveAsImage(std::string imgPath);
		const void saveDiffAsImage(std::string path, Image map);
		

		

		std::string title;
		void const show(const std::string&);
		
	private:
		//
		
		

	};
}
