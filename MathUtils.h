#pragma once

#include <vector>
#include <algorithm>
#include<cmath>
#include<iostream>
#include <Eigen/Dense>
#include<Eigen/Core>
#include <unsupported/Eigen/FFT>
#include<numbers>

namespace mu {

	const double pi = std::numbers::pi;

	template<typename T>
	static T lerp(const T& a, const T& b, double t) {
		return a + t * (b - a);
	}

	// Bilinear interpolation function
	template<typename T>
	static T bilinearInterpolation(const T& topLeft, const T& topRight, const T& bottomLeft, const T& bottomRight,
		double yFraction, double xFraction) {
		// Perform linear interpolation along the top and bottom edges
		T topInterpolation = lerp(topLeft, topRight, xFraction); 
		T bottomInterpolation = lerp(bottomLeft, bottomRight, xFraction);
		T finalInterpolation = lerp(topInterpolation, bottomInterpolation, yFraction);
		// Interpolate along the vertical axis
		return finalInterpolation;
	}
	static void uniqueVec(std::vector<double>& v, double eps) {
		std::vector<int>vi;
		
		std::sort(v.begin(),v.end());
		for (auto& e : v) {
			vi.push_back((int)(e / eps));
		}
		auto ip = std::unique(vi.begin(), vi.end());
		vi.resize(std::distance(vi.begin(), ip));
		int i = 0;
		for (auto& e : vi) {
			v[i++] = e * eps;
		}
		v.resize(vi.size());

	}
	static void getSurroundingPixels(std::array<unsigned int, 2>& topLeft, std::array<unsigned int, 2>& topRight,
		std::array<unsigned int, 2>& bottomLeft, std::array<unsigned int, 2>& bottomRight,
		double yPosition, double xPosition) {
		// Calculate integer positions
		topLeft = { static_cast<unsigned int>(yPosition), static_cast<unsigned int>(xPosition) };
		topRight = { topLeft[0], topLeft[1] + 1};
		bottomLeft = { topLeft[0] + 1, topLeft[1]};
		bottomRight = { bottomLeft[0],topRight[1]};
	}

	static int sign(double n) {
		int sign = -5;
		if (1 / n > 0.0) {
			sign = 1;
		}
		else {
			if (1 / n < 0.0) {
				sign = -1;
			}
		}
		return sign;
	}
	template <typename FloatType>
	static FloatType ceil0(const FloatType& x)
	{
		return _copysign(ceil(abs(x)), x);;
	}
	static double median(std::vector<double> vec) {
		auto m = vec.size() / 2;
		std::nth_element(vec.begin(), vec.begin() + m, vec.end());
		auto med = vec[m];
		if (!(vec.size() & 1)) {
			auto max_it = std::max_element(vec.begin(), vec.begin() + m);
			med = (*max_it + med) / 2.0;
		}
		return med;
	}

	static void unwrapPhaseInPlace(Eigen::VectorXd& in) {
		for (int i = 1; i < in.size(); i++) {
			double d = in[i] - in[i - 1];
			d = d > pi ? d - 2 * pi : (d < -pi ? d + 2 * pi : d);
			in[i] = in[i - 1] + d;
		}
	}

	static int calcGroupDelay(Eigen::VectorXd filter) {
		int N = 90;
		Eigen::VectorXcd fFreq(N);
		Eigen::FFT<double> fft;
		fft.SetFlag(Eigen::FFT<double>::HalfSpectrum);
		fft.fwd(fFreq, filter.matrix().transpose(), N);

		Eigen::VectorXd phase = fFreq.cwiseArg();
		Eigen::VectorXd mags = fFreq.cwiseAbs();
		mags = mags / mags.maxCoeff();
		mags = 20 * mags.array().log10();
		unwrapPhaseInPlace(phase);

		double mag = mags(0);
		int i = 0;
		while (mag < -3) {
			++i;
			mag = mags[i];
		}
		int startIndex = i;
		while (mag > -3) {
			++i;
			mag = mags[i];
		}
		int endIndex = i;


		double ellapsedFrequencies = (endIndex - startIndex) / (1 / pi * fFreq.size());
		double groupDelay = (phase(endIndex) - phase(startIndex)) / ellapsedFrequencies;
		double ezGD = phase(1) * (1 / pi * fFreq.size());
		return ((int) - round(groupDelay));
	}
	static std::array<int, 2> calcGroupDelay(Eigen::MatrixXd kernel) {
		auto svd = kernel.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::VectorXd u = svd.matrixU().leftCols<1>();
		Eigen::VectorXd v = svd.matrixV().leftCols<1>();
		Eigen::VectorXd y = u * svd.singularValues()(0);
		Eigen::VectorXd x = v;	

		int xDelay = calcGroupDelay(x);
		int yDelay = calcGroupDelay(y);
		
		return { yDelay,xDelay };
	}
	static double getPlaneParameterFromNormalAndPoint(Eigen::Vector3d point, Eigen::Vector3d normal) {
		return -normal.dot(point);
	}

	static double angleBetweenVectors(Eigen::Vector3d v1, Eigen::Vector3d v2) {
		double dotProduct = v1.dot(v2);
		dotProduct = std::clamp(dotProduct, -1., 1.);
		double angle = abs(acos(dotProduct) * 180 / pi);

		angle = std::min(angle, 180 - angle);
		return angle;
	}
}