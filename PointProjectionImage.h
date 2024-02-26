#pragma once
#include "Map.h"
namespace lf {
	class LightField;
}
namespace dmap {
	class DisparityMap;
}
namespace ppi {

	
	class PointProjectionImage:public map::Map<cv::Vec3d> {

	private:
		lf::LightField* lightField;
		//std::vector<Vec3b> unoccludedSamples;
		double disparity;
		int m0;
		int n0;
		int kr;
		int lr;
		std::array<int, 2> minView;
		std::array<int, 2> maxView;

		const void getOccludedViews(dmap::DisparityMap& disparityMap, std::vector<std::array<int, 2>>& occlusionsOut);
		const std::array<double, 2> getOcclusionPixel(double dOcc, int k, int l);
		const std::array<double, 2> get4DPPPViewIntersection(double dOcc, double nOcc, double mOcc);
		const bool isOccluded(std::array<int, 2> view, const std::vector<std::array<int, 2>>& occludedViews);


	public:
		PointProjectionImage() = default;
		PointProjectionImage(lf::LightField* lightField, double disparity, uint n0, uint  m0);
		std::array<double, 2> getViewIntersection(int k, int l);
		const void show();
		const double pixelDeviation();
		const double occlusionAwarePixelDeviation(std::vector < std::array<int, 2>> occlusions);
		const double dataCost(dmap::DisparityMap dMap);
		//const bool isOccluded(const std::array<int, 2> view, const std::vector<std::array<int, 2>>& occludedViews);
		
	};

}
