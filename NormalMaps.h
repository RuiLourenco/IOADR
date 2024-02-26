#pragma once
#include "Map.h"
#include "DisparityMap.h"
#include <Eigen/Dense>
#include<Eigen/Core>
namespace nm{
	class Normal {
		Eigen::Vector3d normal;

	public:
		Normal() = default;
		Normal(Eigen::Vector3d normal);
		static Normal fromTangents(Eigen::Vector3d tangentH, Eigen::Vector3d tangentV);
		/**
		 * @brief Provides implicit cast to an Eigen Vector
		 */
		operator Eigen::Vector3d() const;
		/**
		 * @brief getter method
		 */
		const Eigen::Vector3d getNormal();
	};

	class NormalMap : public map::Map<cv::Vec3d> {
	public:
		NormalMap() = default;
		/**
		* @brief Create Normal Map from dimensions
		* @param height height of the normal map
		* @param width width of the normal map
		* @return $RETURN
		*/
		NormalMap(unsigned int height, unsigned int width);
		static NormalMap  estimateNormals(dmap::DisparityMap disparityMap);

		void setNormal(Normal& normal, uint n, uint m);
		cv::Mat calcAngleError(NormalMap map2);
		std::vector<double> angleErrorVec(NormalMap normalMap2, cv::Mat mask);
		double medianAngleError(NormalMap map2, cv::Mat mask);
		Normal getNormal(uint n, uint m);
		NormalMap(cv::Mat_<cv::Vec3d> img);
		/**
		* @brief Show Normals as an RGB triple. Red refers to the y direction, Green the x direction, Blue the z direction.
		*/
		void const show();
		void save(std::string path);
	private:
		/**
		* @brief Return Normal Map as an 8bit unsigned integer openCV matrix.
		* @param $PARAMS
		* @return $RETURN
		*/
		cv::Mat getIntegerNormalizedNormalMap(bool reverseColorChannels);
	};
}
