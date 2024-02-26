#include "MathUtils.h"
#include "IterativeDisparityImprovement.h"
#include "PointProjectionImage.h"
#include "GsImage.h"
#include "Image.h"
#include "LightField.h"
#include <opencv2/photo.hpp>
#include "DiffKernelType.h"
#include <opencv2/core/eigen.hpp>



using namespace idi;

using namespace mapd;
using namespace dmap;



IterativeDisparityImprovement::IterativeDisparityImprovement(lf::LightField& lightField,Parameters parameters, Settings settings):
																												lightField(&lightField),
																												settings(settings),
																												parameters(parameters) {
	this->disparityMap = DisparityMap::fromStructureTensor(lightField, "Iterative Occlusion Aware",parameters.innerScale,parameters.outerScale);
	std::cout << "starting mse = " << disparityMap.mse(15) << std::endl;
	this->cost = Map1d(disparityMap.rows(), disparityMap.cols(), "Current Cost");
	this->evaluation = Map1d(disparityMap.rows(), disparityMap.cols(), "Plane?");
	this->fallBack= Map1d(disparityMap.rows(), disparityMap.cols(), "FallBack?");
	this->centralView = gsImage::GsImage(lightField.getCentralView());
	this->normalMap = nm::NormalMap(disparityMap.rows(), disparityMap.cols());
	this->avgNormalMap = nm::NormalMap(disparityMap.rows(), disparityMap.cols());
	sigmaWeight = parameters.sigma;

	DiffKernelType gType = DiffKernelType::Gaussian;
	DiffKernelType diffType = DiffKernelType::SimpleDiff;
	int support = 2 * parameters.deltaA + 1;
	double filterSigma = (double)support / 1;
	//filters are pre calculated to improve computational efficiency
	this->gEven = gType.getMatrix(support, support, true, false, filterSigma, filterSigma);

	std::array<int, 2> groupDelay = mu::calcGroupDelay(gEven);
	q = 0;

	//The delay through filter translation for the horizontal
	std::array<int, 2> deltaAlg = { 0,parameters.deltaA };
	//shift group delay so it is relative to the central pixel.
	groupDelay[0] = (support - 1) / 2 - groupDelay[0];
	groupDelay[1] = (support - 1) / 2 - groupDelay[1];

	//The Total Group Delay for the horizontal filter relative to the central pixel.
	std::array<int, 2> totalShift = { groupDelay[0] + deltaAlg[0],groupDelay[1] + deltaAlg[1] };

	//The Group delay for the vertical filter will have flipped coordinates as such we approximate the correction as an average of each coordinate
	this->delta = (int) round(totalShift[0] + totalShift[1]) / 2;
	//this->delta = 5;
	//std::cout << "Delta = " << delta << std::endl;
	//n = 0;
	//m = 511;
	//nm::Normal normal = computeEstimatedNormal(disparityMap(n, m));
	//std::cout <<"DISPARITY 30 30 "<< disparityMap(30, 30) << std::endl;
	//calcInitialCost();

	n = nTest;
	m = mTest;
	//double cost = calcDataCost(-0.5784);
	//std::cout << "cost = " << cost << std::endl;
	///*currentAvgPlane = calcEdgeAwarePlaneApproximation();
	//double candidate = getSmoothGeometryCandidate();
	//double disparityChange = abs(candidate - disparityMap(n, m));
	//std::cout << "plane: " << currentAvgPlane.transpose() << std::endl;
	//std::cout << " candidate = " << candidate << " currentDisparity: " << disparityMap(n, m) << std::endl;*/
	//nm::Normal normal = calcEvaluationNormal(1.5);
	//std::cout << "evaluationNormal = "<<normal.getNormal().transpose() << std::endl;
	/*double d = disparityMap(n, m);
	double dataCost = calcDataCost(d);
	double fullCost = calcFullCost(d);*/
	//std::cout << "d = " << d << " data cost = " << dataCost << "full Cost = " << fullCost << std::endl;
	
}
dmap::DisparityMap IterativeDisparityImprovement::run() {
	improveDisparity();
	return disparityMap;
}


bool IterativeDisparityImprovement::checkBoundary(int n_i, int m_i, Eigen::MatrixXd& kernel) {
	//std::cout << "I AM HERE!" << std::endl;
	//std::cout << n_i << "," << m_i << std::endl;
	int oppositeV = n_i + kernel.rows() - 1;
	int oppositeH = m_i + kernel.cols() - 1;
	//std::cout << "opposite:"<<oppositeV << "," << oppositeH << std::endl;
	
	return (disparityMap.isInBounds(n_i, m_i) && disparityMap.isInBounds(oppositeV, oppositeH));

}

const bool IterativeDisparityImprovement::increasingIteration() {
	return !(q % 2);
}
const int IterativeDisparityImprovement::getIncrement() {
	int increment;
	if (increasingIteration()) {
		increment = 1;
	}
	else {
		increment = -1;
	}
	return increment;
}

//void IterativeDisparityImprovement::getInitialPositionAndKernel(int& ni, int& mi, bool forwards, Eigen::MatrixXd& kernel) {
//	
//	bool vertical = kernel.rows() > kernel.cols();
//	if (forwards) {
//		ni = n;
//		mi = m;
//	}
//	else {
//		
//		ni = n - (kernel.rows() -1);
//		mi = m - (kernel.cols() - 1);
//	}
//	//std::cout <<"initial: "<< ni << "," << mi << std::endl;
//	if (ni < 0) {
//		ni = ni + kernel.rows() - 1;
//		if(!vertical) kernel = getDiffKernel(!forwards);
//	}
//	if (ni + kernel.rows() - 1 > disparityMap.rows() - 1) {
//		ni = ni - (kernel.rows() - 1);
//		if (!vertical) kernel = getDiffKernel(!forwards);
//	}
//	if (mi< 0) {
//		mi = mi + kernel.cols() - 1;
//		if (vertical) {
//			kernel = getDiffKernel(!forwards);
//			kernel = kernel.transpose();
//		}
//	}
//	if (mi + kernel.cols() - 1 > disparityMap.cols() - 1) {
//		mi = mi - (kernel.cols() - 1);
//		if (vertical) {
//			kernel = getDiffKernel(!forwards);
//			kernel = kernel.transpose();
//		}
//	}
//}



nm::Normal IterativeDisparityImprovement::computeEstimatedNormalNoBias(double disparityCandidate) {
	

	Eigen::MatrixXd kernelH = this->gEven;
	Eigen::MatrixXd kernelV = this->gEven.transpose();

	int nInit = std::clamp(n - ((int)kernelH.rows()) / 2,0, disparityMap.rows() - (int) kernelH.rows());
	int mInit = std::clamp(m - ((int)kernelH.cols()) / 2,0, disparityMap.rows() - (int) kernelH.cols());

		
	
	nm::Normal normal(Eigen::Vector3d(0, 0, -1));
	if (nInit >= 0 && mInit >= 0 && nInit + kernelH.rows() - 1 < disparityMap.rows() && mInit + kernelH.rows()-1 < disparityMap.rows()) {

		Eigen::Vector3d tangentH(0, 0, 0);
		Eigen::Vector3d tangentV(0, 0, 0);
		getTangentVector(nInit, mInit, disparityCandidate, kernelH, tangentH);
		getTangentVector(nInit, mInit, disparityCandidate, kernelV, tangentV);
		normal = nm::Normal::fromTangents(tangentH, tangentV);
	}
	

	return normal;
}

void IterativeDisparityImprovement::getTangentVector(int n_i, int m_i, double disparityCandidate, Eigen::MatrixXd& kernel, Eigen::Vector3d& tangent) {
	for (int j = 0, currN = n_i; j < kernel.rows(); j++, currN++) {
		for (int i = 0, currM = m_i; i < kernel.cols(); i++, currM++) {
			double currD;
			
			if (currN == n && currM == m) {
				currD = disparityCandidate;
			}
			else {
				currD = this->disparityMap(currN, currM);
			}
			Eigen::Vector3d point = Eigen::Vector3d(lightField->getY(currN, currD), lightField->getX(currM, currD), lightField->getZ(currD));
			double filterWeight = kernel(j, i);

			for (int i = 0; i < tangent.size(); ++i) {
				tangent[i] += filterWeight * point[i];
			}
		}
	}
}

void IterativeDisparityImprovement::updateNormalEstimateNoBias() {
	int increment = getIncrement();
	int nPos = n;
	int mPos = m;


	nm::Normal normal = computeEstimatedNormalNoBias(disparityMap(n, m));
	normalMap.setNormal(normal, nPos, mPos);
}

double IterativeDisparityImprovement::calcDataCost(double d) {
	ppi::PointProjectionImage currImage(lightField, d, n, m);
	double cost;
	if (settings.occlusionAware) {
		cost = currImage.dataCost(this->disparityMap);
	}
	else {
		cost = currImage.pixelDeviation();
	}
	return cost;
}

double IterativeDisparityImprovement::getSmoothedDisparity(double d) {
	CDCParameters params = parameters.cdcParams;
	double fullDispRange = lightField->getDispRange()[1] - lightField->getDispRange()[0];
	const double tauDisp = params.tauDisp*fullDispRange;

	const double epsCost = params.epsDisp + params.rhoCost * cost(n, m);
	//if(inTestPixel())std::cout << "eps Cost = "<<epsCost <<" epsDisp"<< params.epsDisp<<" cost"<< cost(n, m)<<std::endl;
	double smoothnessCost = 0;
	uint nmin = std::max(0, (int)(n - params.deltaCDC));
	uint nmax = std::min((int)(this->disparityMap.rows() - 1), int(n + params.deltaCDC));
	uint mmin = std::max((int)0, (int)(m - params.deltaCDC));
	uint mmax = std::min((int)(this->disparityMap.cols() - 1), (int)(m + params.deltaCDC));
	image::Image centreView = lightField->getView(4, 4);
	cv::Vec3b centerColor = centreView(n, m);

	double newDisp;
	double totalWeight = 0;
	double totalPseudoWeight = 0;
	double pseudoWeightedDisparity = 0;
	double weightedDisparity = 0;
	//double centerTexture = *gsImage.ptr<uchar>(n, m);
	for (uint n0 = nmin; n0 <= nmax; n0++) {
		for (uint m0 = mmin; m0 <= mmax; m0++) {
			if (m0 == m && n0 == n) continue;
			
			cv::Vec3b currentColor = centreView(n0, m0);
			Eigen::Vector3d colorDiff = { (double)currentColor[0] - (double)centerColor[0],(double)currentColor[1] - (double)centerColor[1], (double)currentColor[2] - (double)centerColor[2] };
			double d0 = *this->disparityMap.ptr(n0, m0);
			double deltaDisp = params.rhoDisp * abs(d - d0);
			double deltaColor = params.rhoColor * colorDiff.norm();
			double weight = 0;
			double pseudoWeight = 0;
			//if (inTestPixel())std::cout << "(" << n0 << "," << m0 << ") d = "<<d<<" d0 = "<<d0<<colorDiff.norm() << " " << abs(d - d0);
			//if (inTestPixel())std::cout << "(" << n0 << "," << m0 << ") "<<currentColor << " " << centerColor;
			if (deltaDisp <= tauDisp && deltaColor <= params.tauColor) {
				weight = 1. / std::max(params.epsDisp, sqrt(deltaDisp * deltaDisp + deltaDisp * deltaColor));
				pseudoWeight = 1. / params.epsDisp;
			}
			else if (deltaColor <= params.tauColor) {
				weight = 1. / std::max(epsCost, sqrt(deltaDisp * deltaDisp + deltaColor * deltaColor));
				pseudoWeight = 1. / epsCost;
			}//if (inTestPixel()) std::cout << "(" << n0 << "," << m0 << "): " << weight << " "<<deltaDisp<<" "<<deltaColor<<" ";
			totalWeight += weight;
			totalPseudoWeight += pseudoWeight;
			weightedDisparity += weight * d0;
			pseudoWeightedDisparity += pseudoWeight * d0;
		}
		//if (inTestPixel())std::cout << std::endl;
	}
	double eps = 1e-6;

	newDisp = weightedDisparity / (totalWeight + eps);
	double pseudoNewDisp = pseudoWeightedDisparity / (totalPseudoWeight + eps);
	if (totalWeight == 0) {
		newDisp = -9999;
	}
	//if (inTestPixel())std::cout << "gotcha! "<<tauDisp<<" "<<totalWeight << " " << weightedDisparity << " " << n << "," << m << std::endl;
	return newDisp;
}
double IterativeDisparityImprovement::calcColorDisparityCongruenceCost(double d) {
	double cost;
	double newDisp = getSmoothedDisparity(d);
	if (newDisp == -9999) {
		cost = 0;
	}
	else {
		cost = (d - newDisp) * (d - newDisp);
	}
	
	return cost;
}
const double IterativeDisparityImprovement::currGamma() {
	double gamma;
	if (q >= parameters.qSG) {
		gamma = parameters.gamma0;
	}
	else {
		gamma = 0;
	}
	return gamma;
}
const double IterativeDisparityImprovement::currLambda() {

	double lambda;
	if (q >= parameters.qCdc) {
		lambda = 2*floor(q/parameters.nIter) * parameters.lambda0;
	}
	else {
		lambda = 0;
	}
	return lambda;
}
double IterativeDisparityImprovement::calcSmoothPlaneCost(double d) {
	Eigen::Vector3d averageNormal = currentAvgPlane(Eigen::seq(0,2));
	nm::Normal evaluatedNormal = calcEvaluationNormal(d);
	double angle = mu::angleBetweenVectors(evaluatedNormal, averageNormal);
	if (inTestPixel())std::cout << averageNormal.transpose() << " | " << evaluatedNormal.getNormal().transpose() << " : " << angle << std::endl;;
	double cost = 0;
	double error = std::abs(getSmoothGeometryCandidate() - disparityMap(n, m));
	if (error <= parameters.tau_d) {
		cost = std::min(90., angle);
	}
	return cost;
}
//double IterativeDisparityImprovement::calcSmoothPlaneCost(double d) {
//	double dOpt = getSmoothGeometryCandidate();
//	double cost = abs(dOpt - d);
//	return cost;
//}

const double IterativeDisparityImprovement::getSmoothGeometryCandidate() {

	double zCandidate = lightField->getZFromPlaneAndCentralViewLocation(currentAvgPlane,n,m);
	double dCandidate = lightField->getDisparity(zCandidate);
	return  dCandidate;
}


double IterativeDisparityImprovement::calcFullCost(double d) {	
	double lambda = currLambda();
	double gamma = currGamma();

	double dataCost = calcDataCost(d);
	double cdcCost = 0;
	if (lambda > 0 && settings.useCDC) {
		cdcCost = calcColorDisparityCongruenceCost(d);
	}
	double spCost = 0;
	if (gamma > 0  && abs( getSmoothGeometryCandidate() - disparityMap(n, m)) <= parameters.tau_d && settings.useSP ) {
		spCost = calcSmoothPlaneCost(d);
	}
	//if(inTestPixel()) std::cout << "lambda = " << lambda << " gamma = " << gamma << std::endl;
    if (inTestPixel()) std::cout << " Jdc = " << dataCost << " Jcdc = " << lambda * cdcCost << " Jsp = " << gamma * spCost<<std::endl;
	double fullCost = dataCost + lambda * cdcCost + gamma * spCost;
	return fullCost;
}


void IterativeDisparityImprovement::getIterationLimits(int& nStart, int& mStart, int& nEnd, int& mEnd) {
	if (increasingIteration()) {
		nStart = 0;
		mStart = 0;
		nEnd = this->disparityMap.rows() - 1;
		mEnd = this->disparityMap.cols() - 1;
	}
	else {
		mStart = this->disparityMap.cols() - 1;
		nStart = this->disparityMap.rows() - 1;
		nEnd = 0;
		mEnd = 0;
	}
}

//void IterativeDisparityImprovement::calcInitialCost() {
//	int mStart, nStart,mEnd,nEnd;
//	int increment = getIncrement();
//	getIterationLimits(nStart, mStart, nEnd, mEnd);
//	
//	for (n = nStart; n != nEnd; n += increment) {
//
//		double* d = this->disparityMap.ptr(n);
//		double* c = this->cost.ptr(n);
//		for (m = mStart; m != mEnd; m += increment) {
//
//			updateNormalEstimateNoBias();
//			/*if (currGamma() > 0) {
//				currentAvgPlane = calcEdgeAwarePlaneApproximation();
//			}*/
//			double cost = calcFullCost(*d);
//			*c = cost;
//
//			
//			//accumulatedCost += cost;
//			
//
//			d += increment;
//			c += increment;
//		}
//		
//
//		std::cout << "line: " << n << "     \r";
//	}
//	DisparityMap groundTruth = this->lightField->getGroundTruth();
//	std::cout <<"disparity = "<<disparityMap(nTest,mTest) <<"   "<< "cost = "<<cost(nTest, mTest) <<" Ground Truth Disparity = "<<groundTruth(nTest,mTest)<< std::endl;
//	dmap::DisparityMap gt = lightField->getGroundTruth();
//	std::cout << "mae:" << disparityMap.estimateNormals().medianAngleError(gt.estimateNormals(), lightField->planeMask) << std::endl;
//	std::cout << "maeEstimated:" << normalMap.medianAngleError(gt.estimateNormals(), lightField->planeMask) << std::endl;
//	std::cout << normalMap.getNormal(80, 222).getNormal().transpose() << std::endl;
//	
//	normalMap.show();
//
//
//	//normalMap.show();
//	//disparityMap.show();
//}
bool IterativeDisparityImprovement::inTestPixel() {
	return (n == nTest && m == mTest);
}

//std::vector<double> IterativeDisparityImprovement::getFurtherNeighborCandidates() {
//	std::vector<double> neighborCandidates;
//	std::uniform_int_distribution<int> random(2, 15);
//
//	int n1 = n - random(re);
//	int m1 = m - random(re);
//
//	//std::cout <<"furtherNeighbor Position ("<< n1 <<","<<m1<<")"<< std::endl;
//	//int n1 = n - distance*increment;
//	//int m1 = m - distance*increment;
//	if (n1 > 0 && n1 < (int)disparityMap.rows() - 1) {
//		double d = this->disparityMap(n1, m);
//		neighborCandidates.push_back(d);
//	}
//
//	if (m1 > 0 && m1 < (int)disparityMap.cols() - 1) {
//		double d = this->disparityMap(n, m1);
//		neighborCandidates.push_back(d);
//	}
//	if (n1 > 0 && n1 < (int)disparityMap.rows() - 1 && m1 > 0 && m1 < (int)disparityMap.cols() - 1) {
//		double d = this->disparityMap(n1, m1);
//		neighborCandidates.push_back(d);
//	}
//	return neighborCandidates;
//}


std::vector<double> IterativeDisparityImprovement::getNeighborCandidates() {
	std::vector<double> neighborCandidates;
	int n1 = n - getIncrement();
	int m1 = m - getIncrement();
	//if(inTestPixel())std::cout <<"Neighbor row:"<< n << " " << n1 << std::endl;
	//if(inTestPixel())std::cout <<"Neighbor col:"<< m << " " << m1 << std::endl;
	if (n1 > 0 && n1 < (int)disparityMap.rows() - 1) {
		double d = this->disparityMap(n1, m);
		neighborCandidates.push_back(d);
	}

	if (m1 > 0 && m1 < (int)disparityMap.cols() - 1) {
		double d = this->disparityMap(n, m1);
		neighborCandidates.push_back(d);
	}
	if (n1 > 0 && n1 < (int)disparityMap.rows() - 1 && m1 > 0 && m1 < (int)disparityMap.cols() - 1) {
		double d = this->disparityMap(n1, m1);
		neighborCandidates.push_back(d);
	}
	return neighborCandidates;
}
double IterativeDisparityImprovement::getRandomPerturbationCandidate() {
	//set up RNG distributions
	double sigmaWeight = this->sigmaWeight;
	//std::cout << re.default_seed << std::endl;
	//std::cout << re << std::endl;
	
	double sigma = sigmaWeight * abs(lightField->getDispRange()[1] - lightField->getDispRange()[0]);
	std::normal_distribution<double> norm(0, sigma);
	std::uniform_real_distribution<double> lottery(0, 1);
	//std::cout << "sigma = " << sigma << std::endl;
	double r = norm(re);
	//std::cout << "perturbation: " << r<<std::endl;
	return disparityMap(n,m) + r;
}
void IterativeDisparityImprovement::getAllCandidates() {
	double randomCandidate = withinBounds(getRandomPerturbationCandidate());
	
	candidates.push_back(randomCandidate);
	
	if (currGamma() > 0 && settings.useSPHeuristic) {
		double smoothGeometryCandidate = disparityMap.inRange(getSmoothGeometryCandidate());
		candidates.push_back(smoothGeometryCandidate);
		
		 if(inTestPixel())std::cout << "Smooth Geometry Candidate: " << smoothGeometryCandidate <<std::endl;
	}
	if (currGamma() == 0 && settings.useCDCHeuristic) {
		double cdcCandidate = getSmoothedDisparity(disparityMap(n, m));
		if (cdcCandidate != -9999) {
			candidates.push_back(disparityMap.inRange(cdcCandidate));
			if (inTestPixel())std::cout << "CDC Candidate: " << cdcCandidate << std::endl;
		}
	}
	if (currGamma() == 0 && settings.useSDHeuristic) {
		std::vector<double> neighborCandidates = getNeighborCandidates();
		if (inTestPixel()) {
			std::cout << "Neighbor Candidates:" << std::endl;
			for (auto& nC : neighborCandidates) {
				std::cout << nC << " ";
			}
			std::cout << std::endl;
		}
		candidates.insert(std::end(candidates), std::begin(neighborCandidates), std::end(neighborCandidates));
	}
}

std::array<double, 2> IterativeDisparityImprovement::chooseBestCandidate() {
	std::vector<double> fullCosts;

	double minCost = 999.9e9;
	double minDisparity = 0;
	for (auto d : candidates) {
		//Calc Costs
		double cost = calcFullCost(d);
		if (inTestPixel()) {
			std::cout << d << ": ";
			std::cout << cost << std::endl;
		}

		if (cost < minCost) {
			minCost = cost;
			minDisparity = d;
		}
	}
	
	candidates.clear();
	return { minDisparity,minCost };
}

bool IterativeDisparityImprovement::candidateTest(std::array<double, 2> candidateAndCost) {
	double bestCandidate = candidateAndCost[0];
	double candidateCost = candidateAndCost[1];
	std::uniform_real_distribution<double> lottery(0, 1);
	

	double delta = exp(-(candidateCost - cost(n,m)) / t);
	double randomChance = lottery(re);
	return delta > randomChance;
}

void IterativeDisparityImprovement::updateDataStructures(bool keepCandidate, std::array<double, 2> candidateAndCost) {

	if (keepCandidate) {
		disparityMap(n, m) = candidateAndCost[0];
		cost(n, m) = candidateAndCost[1];
	}
}

double IterativeDisparityImprovement::withinBounds(double disp) {
	double lowerBound = lightField->getDispRange()[0];
	double upperBound = lightField->getDispRange()[1];
	double dynamicRange = upperBound - lowerBound;

	//if (gamma > 0) cout << newDisp << " "<<upperBound<< endl;
	if (disp > 1e5) {
		disp = upperBound;
	}
	if (disp < -1e5) {
		disp = lowerBound;
	}
	double newDisp = disp;
	if (disp > upperBound) {
		double overflow = disp - upperBound;
		int overflowTimes = (int)(overflow / dynamicRange);
		double overflowActual = overflow - dynamicRange * overflowTimes;
		newDisp = upperBound - overflowActual;
	}
	else if (disp < lowerBound) {
		double overflow = newDisp - lowerBound;
		int overflowTimes = (int)(overflow / dynamicRange);
		double overflowActual = overflow - dynamicRange * overflowTimes;
		newDisp = lowerBound - overflowActual;
	}
	return newDisp;
}

void IterativeDisparityImprovement::improveDisparity() {
	//INITIALIZE

	q = 0;
	t = parameters.t0;
	DisparityMap normalDisparity(lightField, "NORMALS GO BOOM");
	//nm::NormalMap evaluationNormals(this->disparityMap.cols(), this->disparityMap.rows());
	//mapd::Map1d choseWell(this->disparityMap.cols(), this->disparityMap.rows(),"chose well?");

	std::cout << "q = " << q << " mse:" << disparityMap.mse(15) << " maePlanes : " << disparityMap.maePlanes() << std::endl;
	
	//main iteration loop
	for (int tIter = 0; tIter < parameters.nTemps; tIter++) {
		for (uint i = 0; i < parameters.nIter; ++i, ++q) {


			// inner cycle
			int mStart, nStart, mEnd, nEnd;
			int increment = getIncrement();
			getIterationLimits(nStart, mStart, nEnd, mEnd);
			if (q > parameters.qSG) {
				sigmaWeight = 0.01 * parameters.sigma;
			}

			double accumDataCost = 0;
			for (n = nStart; n != nEnd; n += increment) {
				double* d = this->disparityMap.ptr(n) + mStart;
				double* c = this->cost.ptr(n) + mStart;
				for (m = mStart; m != mEnd; m += increment) {
					//Necessary as the cost function will change between iterations.
					cost(n, m) = calcFullCost(disparityMap(n, m));
					if (q >= parameters.qCdc - 1) {
						updateNormalEstimateNoBias();
					}
					if (currGamma() > 0) {
						currentAvgPlane = calcEdgeAwarePlaneApproximation();
						avgNormalMap(n, m) = { currentAvgPlane(0), currentAvgPlane(1), currentAvgPlane(2) };
						double candidate = getSmoothGeometryCandidate();
						evaluation(n, m) = calcSmoothPlaneCost(candidate);
						normalDisparity(n, m) = candidate;
						//double disparityChange = abs(candidate - disparityMap(n, m));
						//if (inTestPixel())std::cout << "candidate disparity : " << candidate << "  " << disparityChange << std::endl;
					}
					
					//std::cout << "(" << n << "," << m << ") " << *d << ": " <<*c << std::endl;
					//std::cout << std::endl;

					
					
					if (inTestPixel())std::cout << disparityMap(n, m) << ": " << cost(n, m) << std::endl;
					getAllCandidates();
					if (inTestPixel())std::cout << "candidates: ";
					for (auto& candidate : candidates) {
						if (inTestPixel())std::cout << candidate << " ";
					}
					if (inTestPixel())std::cout << std::endl << std::endl;
					std::array<double, 2> candidateAndCost = chooseBestCandidate();
					bool keepCandidate = candidateTest(candidateAndCost);
					updateDataStructures(keepCandidate, candidateAndCost);
					if (q >= parameters.qCdc - 1) {
						updateNormalEstimateNoBias();
					}
					if (inTestPixel())std::cout << "d = " << disparityMap(n, m)<<" "<<"c = "<<cost(n,m)<<std::endl;
					if (inTestPixel())std::cout << std::endl;

					d += increment;
					c += increment;

				}
				std::cout << "line: " << n << "   " << "\r";

			}
			
			std::cout << "q = " << q << " mse:" << disparityMap.mse(15) << " badpix = " << disparityMap.badPix(0.07) << " maePlanes:" << disparityMap.maePlanes() << std::endl;
			disparityMap.show();
		}
		t = parameters.t0 * pow(parameters.alpha, tIter + 1);
	}


}

void IterativeDisparityImprovement::getWindowLimits(int& n0, int& nMax, int& m0, int& mMax, int radius) {

	n0 = std::max((int)(n - radius), parameters.deltaA*2+1);
	nMax = std::min((int)(n + radius), (int)(this->disparityMap.rows() - 1-(parameters.deltaA*2+1)));
	m0 = std::max((int)(m - radius), parameters.deltaA*2+1);
	mMax = std::min((int)(m + radius), (int)(this->disparityMap.cols() - 1 - (parameters.deltaA*2+1)));
}
//void IterativeDisparityImprovement::get	WindowLimits(int& n0, int& nMax, int& m0, int& mMax, int radius) {
//	if (increasingIteration()) {
//		n0 = std::max((int)(n - radius), parameters.deltaA * 2 + 1);
//		nMax = std::min((int)(n ), (int)(this->disparityMap.cols() - 1 - (parameters.deltaA * 2 + 1)));
//		m0 = std::max((int)(m - radius), parameters.deltaA * 2 + 1);
//		mMax = std::min((int)(m), (int)(this->disparityMap.rows() - 1 - (parameters.deltaA * 2 + 1)));
//	}
//	else {
//		n0 = std::max((int)(n), parameters.deltaA * 2 + 1);
//		nMax = std::min((int)(n+radius), (int)(this->disparityMap.cols() - 1 - (parameters.deltaA * 2 + 1)));
//		m0 = std::max((int)(m), parameters.deltaA * 2 + 1);
//		mMax = std::min((int)(m+radius), (int)(this->disparityMap.rows() - 1 - (parameters.deltaA * 2 + 1)));
//	}
//	
//}

Eigen::Vector4d IterativeDisparityImprovement::calcEdgeAwarePlaneApproximation() {
	double currDisparity = this->disparityMap(n, m);
	int n0, nMax, m0, mMax;
	getWindowLimits(n0, nMax, m0, mMax, this->parameters.deltaAvg);
	if (n < n0 || n >nMax || m < m0 || m >mMax) {
		Eigen::Vector3d finalNormal = this->normalMap.getNormal(n,m);
		double finalA = mu::getPlaneParameterFromNormalAndPoint(lightField->getPoint(n, m, currDisparity), finalNormal);
		return { finalNormal(0),finalNormal(1),finalNormal(2),finalA };
	
	}
	
	std::vector<nm::Normal> neighborNormals;
	std::vector <std::array<int, 2>> coords;
	std::vector<nm::Normal> normalsInS;

	std::vector<std::array<int, 2>> coordsInR;
	nm::Normal currNormal = this->normalMap.getNormal(n, m);
	std::vector<double> angleBeta;


	
	nm::NormalMap avgWindow(parameters.deltaAvg * 2 + 1, parameters.deltaAvg * 2 + 1);
	for (int nCurr = n0; nCurr <= nMax; nCurr++) {
		for (int mCurr = m0; mCurr <= mMax; mCurr++) {
			if (nCurr == n && mCurr == m) continue;

			
			nm::Normal normal = this->normalMap.getNormal(nCurr, mCurr);
			avgWindow.setNormal(normal,nCurr - n0, mCurr - m0);
			double disparity = this->disparityMap(nCurr, mCurr);
			double angle = mu::angleBetweenVectors(normal, currNormal);
			neighborNormals.push_back(normal);
			angleBeta.push_back(angle);
			coords.push_back({ nCurr,mCurr });
		}
	}
	//std::cout << "There are a total of " << coords.size() << " normals in the average window!" << std::endl;
	//if (inTestPixel())normalMap.show();
	//if(inTestPixel())avgWindow.show();
	image::Image hasInlierNormal(parameters.deltaAvg*2+1, parameters.deltaAvg * 2 + 1,"Inlier Normals");
	image::Image hasInlierPlane(parameters.deltaAvg * 2 + 1, parameters.deltaAvg * 2 + 1,"Inlier Plane");
	double acc = 0;
	Eigen::Vector3d averageNormal = Eigen::Vector3d::Zero();
	double average = std::reduce(angleBeta.begin(), angleBeta.end()) / angleBeta.size();
	double threshold = parameters.tau_k * average;
	std::vector <std::array<int, 2>> coordsInS;
	if (inTestPixel()) std::cout <<currNormal.getNormal().transpose()<< "average:" << average << " " << parameters.tau_k * average << std::endl;
	for (int i = 0; i < angleBeta.size(); ++i) {
		if (inTestPixel()) {
			//std::cout << "(" << coords[i][0] << "," << coords[i][1] << ")  " << neighborNormals[i].getNormal().transpose() << " angle: " << angleBeta[i] << std::endl;
		}
		if (angleBeta[i] < threshold) {
			normalsInS.push_back(neighborNormals[i]);
			averageNormal += neighborNormals[i].getNormal();
			coordsInS.push_back(coords[i]);
			if(inTestPixel())hasInlierNormal(coords[i][0]-n0, coords[i][1]-m0) = 255;
			
		}
	}
	//std::cout << "There are a total of " << normalsInS.size() << " normals that were considered inliers, anglewise!" << std::endl;

	averageNormal /= normalsInS.size();
	averageNormal = averageNormal.normalized();
	//std::cout << "THE SIGMA AVERAGE NORMAL IS :  " << averageNormal.transpose() << std::endl;
	//if (inTestPixel())hasInlierNormal.show();


	std::vector<double> estimatedDisparities;
	Eigen::Vector3d finalNormal = Eigen::Vector3d::Zero();
	double finalA = 0;
	int inlierNeighborCount = 0;
	//if (inTestPixel())std::cout << "current disparity = "<<currDisparity<< std::endl;

	for (int i = 0; i < normalsInS.size(); ++i) {
		double disparity = disparityMap(coordsInS[i][0], coordsInS[i][1]);
		
		Eigen::Vector3d point = lightField->getPoint(coordsInS[i][0], coordsInS[i][1], disparity);
		double a = mu::getPlaneParameterFromNormalAndPoint(point, averageNormal);

		double zEstimate = lightField->getZFromPlaneAndCentralViewLocation({ averageNormal(0),averageNormal(1),averageNormal(2),a }, n, m);
		double dEstimate = lightField->getDisparity(zEstimate);
		
		
		//if (inTestPixel())std::cout << "(" << coordsInS[i][0] << "," << coordsInS[i][1] << ") = " << abs(dEstimate - currDisparity) << " " << dEstimate << std::endl;

		if (abs(dEstimate - currDisparity) < parameters.tau_d) {
			//if (inTestPixel())std::cout << "(" << coordsInS[i][0] << "," << coordsInS[i][1] << ") = " << disparity << " " << a << std::endl;
		    //if (inTestPixel())std::cout << "(" << coordsInS[i][0] << "," << coordsInS[i][1] << ") = " << abs(dEstimate - currDisparity) << " " << dEstimate <<" "<<currDisparity<< " | " << finalNormal.transpose() << std::endl;
			coordsInR.push_back(coordsInS[i]);
			if(inTestPixel())hasInlierPlane(coordsInS[i][0] - n0, coordsInS[i][1] - m0) = 255;
			//finalNormal += currNormal.getNormal();
			finalA += a;
			inlierNeighborCount++;
		}
	}

	

	//if(inTestPixel())std::cout << "Only " << inlierNeighborCount << " normals were considered inliers when accounting for z!" << std::endl;
	
	//if (inTestPixel()) hasInlierPlane.show();
	//double aFinal = 0;
	if (inlierNeighborCount >0) {
		//finalNormal /= inlierNeighborCount;
		finalA /= inlierNeighborCount;
	    finalNormal = averageNormal;
		double norm = finalNormal.norm();
		//finalNormal /= norm;
		/*for (int i = 0; i < coordsInR.size(); ++i) {
			double disparity = disparityMap(coordsInR[i][0], coordsInR[i][1]);
			Eigen::Vector3d point = lightField->getPoint(coordsInR[i][0], coordsInR[i][1], disparity);
			double a = mu::getPlaneParameterFromNormalAndPoint(point, finalNormal);
			aFinal += a;
			double zEstimate = lightField->getZFromPlaneAndCentralViewLocation({ averageNormal(0),averageNormal(1),averageNormal(2),a }, n, m);
			double dEstimate = lightField->getDisparity(zEstimate);
		}

		aFinal /= coordsInR.size();*/
		if (inTestPixel()) {
			double zEstimate = lightField->getZFromPlaneAndCentralViewLocation({ averageNormal(0),averageNormal(1),averageNormal(2),finalA }, n, m);
			double dEstimate = lightField->getDisparity(zEstimate);
			std::cout << "current normal = " << currNormal.getNormal().transpose() <<" || finalNormal = " <<finalNormal.transpose() << std::endl;
			std::cout << "d = " << dEstimate << std::endl;
			
		}
	}
	else {
		fallBack(n, m) = 555;
		finalNormal = averageNormal;
		finalA = mu::getPlaneParameterFromNormalAndPoint(lightField->getPoint(n, m, currDisparity), finalNormal);
	}
	return { finalNormal(0),finalNormal(1),finalNormal(2),finalA};
}

nm::Normal IterativeDisparityImprovement::calcEvaluationNormal(double d) {
	double delta = -getIncrement() * 2;
	//double delta = -getIncrement() * 1;
	int n1, m1,n2,m2;
	n1 = n + delta;
	m1 = m + delta;
	n2 = n - delta;
	m2 = m - delta;
	if (n1 < 0 || n1 > disparityMap.rows()-1) {
		n1 = n2;
	}
	if (m1 < 0 || m1 > disparityMap.cols()-1) {
		m1 = m2;
	}


	Eigen::Vector3d pointCurrent = lightField->getPoint(n, m, d);
	Eigen::Vector3d pointVertical = lightField->getPoint(n1, m, disparityMap(n1,m));
	Eigen::Vector3d pointHorizontal = lightField->getPoint(n, m1, disparityMap(n,m1));
	Eigen::Vector3d horizontalTangent = pointCurrent - pointHorizontal;
	Eigen::Vector3d verticalTangent = pointCurrent - pointVertical;
	nm::Normal normal = nm::Normal::fromTangents(verticalTangent, horizontalTangent);

	if (inTestPixel()) {
		std::cout << "(" << n << "," << m << ") " << d << " " << "(" << n1 << "," << m << ") " << disparityMap(n1, m) << " ""(" << n << "," << m1 << ") " << disparityMap(n, m1) << std::endl;
		std::cout << normal.getNormal().transpose() << std::endl;
	}

	return normal;
}

//DisparityMap IterativeOcclusionAwareOptimizer::estimateDisparity() {
//
//	calcCurrentCost();
//	iterativeDisparityImprovement();
//	return disparityMap;
//}
