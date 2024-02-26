#include "PointProjectionImage.h"
#include "LightField.h"
#include "Image.h"
#include "DisparityMap.h"
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "MathUtils.h"

using namespace ppi;
using namespace lf;
using namespace cv;
using namespace std;

PointProjectionImage::PointProjectionImage(LightField* lightField,double d,uint n0, uint m0) {
	this->lightField = lightField;
	this->n0 = n0;
	this->m0 = m0;
	this->disparity = d;
	//This version of the software only supports the central view as the reference view.
	this->lr = lightField->viewMatrixHeight() / 2;
	this->kr = lightField->viewMatrixWidth() / 2;

	
	
	double lDeltaMax = lr, kDeltaMax=kr, lDeltaMin=lr, kDeltaMin=kr; 
	if (abs(d)<0.001) {
		lDeltaMax = 8;
		kDeltaMax = 8;
		lDeltaMin = 0;
		kDeltaMin = 0;
	}
	else {
		if (d < 0) {
			lDeltaMax = (lightField->viewHeight() - 1 - n0) / abs(d);
			kDeltaMax = (lightField->viewWidth() - 1 - m0) / abs(d);
			lDeltaMin = n0 / abs(d);
			kDeltaMin = m0 / abs(d);
		}if (d > 0) {
			lDeltaMin = (lightField->viewHeight() - 1 - n0) / abs(d);
			kDeltaMin = (lightField->viewWidth() - 1 - m0) / abs(d);
			lDeltaMax = n0 / abs(d);
			kDeltaMax = m0 / abs(d);
		}
	}
	

	

	int lMax = std::min(lightField->viewMatrixHeight()-1,(int)std::floor(this->lr + lDeltaMax));
	int kMax = std::min(lightField->viewMatrixHeight()-1,(int)std::floor(this->kr + kDeltaMax));
	int lMin = std::max(0,(int)std::ceil(this->lr - lDeltaMin));
	int kMin = std::max(0,(int)std::ceil(this->kr - kDeltaMin));
	if (lMax < 0 || kMax < 0) {
		cout << "WHAT THÈ HELL IS GOING ON?" << endl;
		cout << disparity << endl;
	}
	//cout << std::floor(this->kr - kDeltaMin) << " " << std::ceil(this->kr - kDeltaMin) <<" "<< (this->kr - kDeltaMin) <<" "<<kMin<< endl;
	this->minView = { lMin,kMin };
	this->maxView = { lMax,kMax };
	//cout << "lDeltaMax = " << lDeltaMax << " kDeltaMax = " << kDeltaMax << " lDeltaMin = " << lDeltaMin << " kDeltaMin = " << kDeltaMin << endl;
	//cout << "Limits: (" << lMin << "," << kMin << ") (" << lMax << "," << kMax << ")" << endl;
	
	//Initialize the 4D-PPI as a zero matrix.
	this->data = Mat(lightField->viewMatrixHeight(), lightField->viewMatrixWidth(), CV_8UC3, Scalar(0, 0, 0));
	for (int l = lMin; l<=lMax; l++) {
		for (int k = kMin; k <= kMax; k++) {
			image::Image view = lightField->getView(l, k);
			array<double, 2> intersection = this->getViewIntersection(l,k);
			if (view.isInBounds((int)ceil(intersection[0]), (int)ceil(intersection[1]))) {
				(*this)(l, k) = view.getValue(intersection[0], intersection[1]);
				//cout << (*this)(l, k);
				//cout <<" [ "<< intersection[0] <<"  "<<intersection[1]<< "] | ";
			}
			else {
				/*cout << "something isn't quite correct" << endl;
				cout << "("<<l<<","<<k<<")" << "(" << n0 << "," << m0 << ")" << endl;
				cout << "(" << lMax << "," << kMax << ")" << "(" << lMin << "," << kMin << ")" << "(" << intersection[0] << "," << intersection[1] << ")" << endl;
				cout << d << endl;
				cout << lDeltaMax <<" "<<lDeltaMin<<" "<<d << endl;
				(*this)(l, k) = 0;*/
			}
		}
		//cout << endl;
	}
	//cout << endl;
	//cout << endl;
	//cout << endl;
}

const bool PointProjectionImage::isOccluded(const array<int, 2> view, const std::vector<std::array<int, 2>>& occludedViews) {
	bool occluded = false;
	for (auto occludedView : occludedViews) {
		if (occludedView[0] == view[0] && occludedView[1] == view[1]) {
			occluded = true;
		}
	}
	return occluded;
}
const void PointProjectionImage::show() {
	cv::Mat viz;
	cv::resize(this->data, viz, cv::Size(), 20, 20, cv::INTER_NEAREST_EXACT);
	viz.convertTo(viz, CV_8UC1);
	std::string windowName = "4dppi"; //Name of the window
	cv::namedWindow(windowName);
	cv::imshow(windowName, viz);
	cv::waitKey(0);
}

const void PointProjectionImage::getOccludedViews(dmap::DisparityMap& disparityMap, vector<array<int, 2>>& occlusionsOut) {
	array<double, 2> dispRange = this->lightField->getDispRange();
	int windowSize = (int)floor((dispRange[1] - this->disparity) * (max(this->lr, this->kr)+0.5));
	int nStart = max(this->n0 - windowSize, 0);
	int mStart = max(this->m0 - windowSize, 0);
	int nEnd = min(this->n0 + windowSize, this->lightField->viewHeight()-1);
	int mEnd = min(this->m0 + windowSize, this->lightField->viewWidth()-1);
	//cout << nStart << "," << mStart << "   " << nEnd << "," << mEnd << endl;

	double tolerance = 0.05;
	//cout << "windowSize = "<< windowSize << endl;
	//Build vector with all disparities that could be potential occluders
	vector<double> occludingDisparities;
	for (int n = nStart; n <= nEnd; n++) {
		for (int m = mStart; m <= mEnd; m++) {
			double d1 = disparityMap(n, m);
			//cout << d1 << endl;
			if (this->disparity - d1 < -0.01) {
				//cout << "hello: " <<d1<< endl;
				occludingDisparities.push_back(d1);
			}
		}
	}
	//eliminate duplicates
	mu::uniqueVec(occludingDisparities,tolerance);
	//cout << "Unique:" << endl;
	//for (auto& occ : occludingDisparities) cout << occ << endl;
	//cout << endl;
	//half pixel allowed threshold
	double threshold = 0.5;
	//Iterate over each view of the 4D-PPP
	for (int l = minView[0]; l <= maxView[0]; l++) {
		for (int k = minView[1]; k <= maxView[1]; k++) {
			bool occluded = false;
			if (k == kr && l == lr) {
				continue;
			}
			for (double dOcc : occludingDisparities) {
				if (occluded == true) {
					continue;
				}
				//get parameter mOcc of occluding 4D-PPP ie. the position where it intersects view (lr,kr)
				std::array<double, 2> occluderPosition = getOcclusionPixel(dOcc, l,k);
				//If it's within bounds
				if (disparityMap.isInBounds((int)mu::ceil0(occluderPosition[0]), (int)mu::ceil0(occluderPosition[1]))) {
					//occlusion disparity according to the disparity map
					//std::cout << occluderPosition[0] << "," << occluderPosition[1] << std::endl;
					double modelDispOcc = disparityMap.getValue(occluderPosition[0], occluderPosition[1]);
					std::array<double, 2> viewIntersection = get4DPPPViewIntersection(modelDispOcc, occluderPosition[0], occluderPosition[1]);
					//DEBUG

					
					if (isfinite(viewIntersection[0]) && isfinite(viewIntersection[1])) {
						double errorL = fabs(viewIntersection[0] - l);
						double errorK = fabs(viewIntersection[1] - k);
						//cout << l << "," << k << "  " << viewIntersection[0] << " " << viewIntersection[1] << " "<< dOcc<<endl;
						//cout << occluderPosition[0] << "," << occluderPosition[1] << "  " << modelDispOcc << endl;
						//cout << errorL << " " << errorK << " < " << threshold << endl;
						//if (l == 3 && k == 4 && dOcc == 0.15) {
						//	cout << l << "," << k << "  " << viewIntersection[0] << " " << viewIntersection[1] << " "<< dOcc<<endl;
						//	cout << occluderPosition[0] << "," << occluderPosition[1] << "  " << modelDispOcc << endl;
						//}
					
						if (errorL < threshold && errorK < threshold) {
							std::array<int, 2> view = { l,k };
							occlusionsOut.push_back(view);
							occluded = true;
						}
					}
				}
			}
		}
	}
}

const std::array<double,2> PointProjectionImage::get4DPPPViewIntersection( double dOcc, double nOcc, double mOcc) {
	double lOcc = ((double) n0 - nOcc) / (disparity- dOcc) + (double)lr;
	double kOcc = ((double)m0 - mOcc) / (disparity - dOcc) + (double)kr;
	return { lOcc,kOcc };
}

const std::array<double, 2> PointProjectionImage::getOcclusionPixel(double dOcc, int l, int k) {
	double n = (dOcc - disparity) * ((double)l - lr) + n0;
	double m = (dOcc - disparity) * ((double)k - kr) + m0;
	return {n,m};
}

array<double, 2> PointProjectionImage::getViewIntersection(int l, int k) {
	double n = -disparity * (double(l - lr)) + n0;
	double m = -disparity * (double(k - kr)) + m0;
	return {n,m};
}

const double PointProjectionImage::pixelDeviation() {
	Vec3d centreValue = data(4, 4);
	//cout << "Centre Value: "<<(int)centreValue[0] <<","<< (int)centreValue[1]<<","<< (int)centreValue[2] << endl;
	Vec3d summ = { 0,0,0 };
	int count = 0;
	for (int i = minView[0]; i <= maxView[0]; ++i) {
		Vec3d* row_ptr = data.ptr<Vec3d>(i);
		for (int j = minView[1]; j <= maxView[1]; ++j) {
			Vec3d deviation = row_ptr[j] - centreValue;
			Vec3d absoluteDeviation = { abs(deviation[0]), abs(deviation[1]), abs(deviation[2]) };
			summ += absoluteDeviation;
			++count;
		}
	}
	double cost = (summ[0] + summ[1] + summ[2]) / 3;
	cost /= (count-1);
	
	return cost;

}



const double PointProjectionImage::occlusionAwarePixelDeviation(std::vector<std::array<int,2>> occlusions) {
	
	Vec3d centreValue = data(4, 4);
	Vec3d summ = { 0,0,0 };
	int count = 0;
	//cout << occlusions.size() << " occlusions were found: " << endl;
	//for (auto occ : occlusions)cout << occ[0] << " " << occ[1] << endl;
	for (int i = minView[0]; i <= maxView[0]; ++i) {
		Vec3d* row_ptr = data.ptr<Vec3d>(i);
		for (int j = minView[1]; j <= maxView[1]; ++j) {
			Vec3d deviation = row_ptr[j] - centreValue;
			Vec3d absoluteDeviation = { abs(deviation[0]), abs(deviation[1]), abs(deviation[2]) };
			if (!isOccluded({ i,j }, occlusions)) {
				summ += absoluteDeviation;
				++count;
			}
		}
	}
	if(count > 0){
		summ /= (count-1);
	}
	else {
		cerr << this->n0 << "," << this->m0 << endl;
		cerr << "minView = "<<minView[0] << " " << minView[1] << endl;
		cerr << "maxView = "<<maxView[0] << " " << maxView[1] << endl;
		for (auto occ : occlusions)cout <<"("<< occ[0] << " " << occ[1] << ") ";
		cout << endl;
		cerr << "occluded centre view!!!!" << endl;
		return -1;
	}
	return (summ[0] + summ[1] + summ[2]) / 3;

}

const double PointProjectionImage::dataCost(dmap::DisparityMap dMap) {
	vector<array<int, 2>> occlusions;
	getOccludedViews(dMap, occlusions);
	double jPd = pixelDeviation();
	double jOapd = occlusionAwarePixelDeviation(occlusions);
	double jDc = 0;
	
	for (auto views : occlusions) {
		//cout << "OCCLUSION: " << views[0] << " , " << views[1] << endl;
	}

	int views = (this->maxView[0] - this->minView[0] + 1) * (this->maxView[1] - this->minView[1] + 1);

	if (views - occlusions.size() < 0.07 * views) {
		jDc = jPd;
	}
	else {
		jDc = min(jPd, jOapd);
	}
	//std::cout << views - occlusions.size() << "  <  " << 0.07 * views << endl;
	//std::cout << "J_pd = "<<jPd << " J_oapd = " << jOapd << " J_dc = " << jDc << std::endl;
	return jDc;
}
