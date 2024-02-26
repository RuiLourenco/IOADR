#pragma once
#include <cstdint>
#include <numeric>
#include <algorithm>
#include <string>
#include <vector>
#include <array>
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include<Eigen/Core>


class DiffKernelType {

public:
	enum Value : uint8_t {
		Gaussian,
		SimpleDiff,
		Scharr,
		ScharrOuter,
		Sobel,
		none
	};
	DiffKernelType() = default;
	DiffKernelType(std::string);
	constexpr DiffKernelType(Value aCost) : value(aCost) {}


	// Allow switch and comparisons.
	constexpr operator Value() const { return value; }
	// Prevent usage: if(fruit)
	explicit operator bool() const = delete;
	std::string toString();
	Eigen::MatrixXd  getMatrix(int height, int width, bool topAverage, bool half, double sigmaX, double sigmaY);

	Eigen::MatrixXd getScharr(int height, int width, bool top, bool half);
	Eigen::MatrixXd getSimpleDiff();
	Eigen::MatrixXd getScharrOuter(int height, int width, bool top, bool half);
	Eigen::MatrixXd getSobel(int height, int width, bool top, bool half);
	Eigen::MatrixXd getGaussianDerivativeMatrix(int height, int width, bool top, bool half, double sigmaX, double sigmaY);
	Eigen::MatrixXd getGaussianDerivativeMatrix(int height, int width, double sigmaX, double sigmaY);
	Eigen::ArrayXd gaussian(Eigen::ArrayXd x, double mean, double std);
	Eigen::ArrayXd gaussianDerivative(Eigen::ArrayXd x, double mean, double std);
	void splitKernel(Eigen::MatrixXd& kernel, bool top);



private:
	Value value;
};
