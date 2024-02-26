#pragma once
#pragma once
#include "Map.h"
#include <opencv2/core.hpp>


namespace image {
	class Image;
}
namespace gsImage {
	class GsImage :public map::Map<uchar>
	{
	public:
		GsImage() = default;
		GsImage(const image::Image& colourImage);
		GsImage(unsigned int height, unsigned int width);
		GsImage(cv::Mat_<unsigned char> img);
		GsImage(unsigned int height, unsigned int width, std::string);
		GsImage(cv::Mat_<unsigned char> img, std::string);
		GsImage(std::string);

		const double getValue(double n, double m);
		//cv::Vec3b operator()(double n, double m);

		void const show();
		const void saveAsImage(std::string imgPath);




		std::string title;
		void const show(const std::string&);

	private:
		//



	};
}
