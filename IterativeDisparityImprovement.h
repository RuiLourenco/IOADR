#pragma once
#include "DisparityMap.h"
#include "Map1d.h"
#include "GsImage.h"
#include "MathUtils.h"
#include "NormalMaps.h"
#include "Parameters.h"
#include <random>
namespace lf {
	class LightField;
}
namespace idi {
	
	class IterativeDisparityImprovement {
		/* Pointer to the Light Field for which disparity is being estimated. */
		lf::LightField* lightField;
		/* The central view of the light field as a gray scale image. */
		gsImage::GsImage centralView;
		/* Structure containing all the working parameters of the algorithm. */
		Parameters parameters;
		/* Structure containing all the working parameters of the algorithm. */
		Settings settings;
		/* The working disparity map being refined */
		dmap::DisparityMap disparityMap;
		/* The current cost for each pixel */
		mapd::Map1d cost;
		/* The current estimated normal map */
		nm::NormalMap normalMap;
		/* Difference of Gaussian Matrix used for even refinement iterations */
		Eigen::MatrixXd gEven;
		///* Difference of Gaussian Matrix used for odd refinement iterations */
		//Eigen::MatrixXd gOdd;
		///* Difference Matrix used for evaluating surface normal accuracy*/
		//Eigen::MatrixXd gDiff;
		/* The current iteration*/
		int q;
		/*The Average Normal of neighbors centered around the current pixel*/
		Eigen::Vector4d currentAvgPlane = Eigen::Vector4d::Zero();

		double sigmaWeight;
		
		/* The delay of the estimated normal map relative to the algorithm*/
		int delta;
		/* Current Vertical Position */
		int n = 0;
		/* Current Horizontal Position */
		int m = 0;
		/*Current Temperature*/
		double t = 0;
		/*Candidate Vector*/
		std::vector<double> candidates;
		/*random number generator engine*/
		std::default_random_engine re;
		double withinBounds(double disp);
		/* For Debug Purposes Only, Should be Deleted before official publication*/
		int nTest = -1;
		int mTest = -1;
		//int nTest = 239;
		//int mTest = 50;

		bool inTestPixel();
		mapd::Map1d evaluation;
		mapd::Map1d fallBack;
		nm::NormalMap avgNormalMap;
		double averageCost;
		

		
		void updateNormalEstimateNoBias();
		void updateNormalEstimate();
		void improveDisparity();
		void getAllCandidates();
		std::array<double, 2> chooseBestCandidate();
		std::vector<double> getNeighborCandidates(); 
		std::vector<double> getFurtherNeighborCandidates(); 
		bool candidateTest(std::array<double, 2> candidateAndCost);
		void updateDataStructures(bool keepCandidate, std::array<double, 2> candidateAndCost);
		double getRandomPerturbationCandidate();
		void getIterationLimits(int& nStart, int& mStart, int& nEnd, int& mEnd);
		const double currGamma();
		const double currLambda();
		const bool increasingIteration();
		const int getIncrement();
		bool checkBoundary(int n_i, int m_i, Eigen::MatrixXd& kernel);
		void calcInitialCost();
		void getTangentVector(int n_i, int m_i, double disparity, Eigen::MatrixXd& kernel, Eigen::Vector3d& tangent);
		nm::Normal computeEstimatedNormal(double disparityCandidate);
		nm::Normal computeEstimatedNormalNoBias(double disparityCandidate);
		void getInitialPositionAndKernel(int& ni, int& mi, bool forwards, Eigen::MatrixXd& kernel);
		Eigen::MatrixXd getDiffKernel(bool forward);
		double calcFullCost(double d);
		double calcDataCost(double d);
		double calcSmoothPlaneCost(double d);
		double getSmoothedDisparity(double d);
		double calcColorDisparityCongruenceCost(double d);
		void getWindowLimits(int& n0, int& nMax, int& m0, int& mMax, int radius);
		Eigen::Vector4d calcEdgeAwarePlaneApproximation();
		nm::Normal calcEvaluationNormal(double d);
		const double getSmoothGeometryCandidate();
	public:
		IterativeDisparityImprovement(lf::LightField&, Parameters parameters,Settings settings = Settings());
		//IterativeDisparityImprovement(lf::LightField&, Parameters parameters);
		dmap::DisparityMap run();
	};
}