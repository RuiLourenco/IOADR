#pragma once
#include "DisparityMap.h"
#include <opencv2/core.hpp>

namespace lf {
	class LightField;
}
namespace mapd{
	class Map1d;
}
namespace image {
	class Image;
}
namespace  ste {
	class StructureTensorEstimator {
		lf::LightField* lightField;
		double innerScale;
		double outerScale;


		const void disparityFromEPI(image::Image& inputEPI, cv::Mat& disparityOut, cv::Mat& trustOut);
		const void estimateDisparity(dmap::DisparityMap& disparityOut, mapd::Map1d& trust, bool isHorizontal);
		const void mergeDisparities(dmap::DisparityMap& horizontalDisp,
									dmap::DisparityMap& verticalDisp,
									mapd::Map1d& horizontalTrust,
									mapd::Map1d& verticalTrust,
									dmap::DisparityMap& finalDisp,
									mapd::Map1d& finalTrust);

	public:
		StructureTensorEstimator() = default;
		StructureTensorEstimator(lf::LightField&, double innerScale, double outerScale);

		const dmap::DisparityMap estimateDisparity();

	};


}
