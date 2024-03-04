#include "StructureTensorEstimator.h"
#include "LightField.h"
#include "DiffKernelType.h"
#include "MathUtils.h"
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>

#include "Image.h"
#include "Map1d.h"


using namespace ste;
using namespace lf;
using namespace dmap;
using namespace mapd;
using namespace Eigen;
using namespace cv;
using namespace mapd;

StructureTensorEstimator::StructureTensorEstimator(LightField& lightField,double is, double os): lightField(&lightField),innerScale(is),outerScale(os){

}

const DisparityMap StructureTensorEstimator::estimateDisparity() {
	
	DisparityMap tempHorizontal(lightField,"horizontal disparity");
	Map1d tempHorizontalTrust(lightField->viewHeight(), lightField->viewWidth(),"horizontal disparity trust");
	DisparityMap tempVertical(lightField, "vertical disparity");
	Map1d tempVerticalTrust(lightField->viewHeight(), lightField->viewWidth(), "vertical disparity trust");

	estimateDisparity(tempHorizontal, tempHorizontalTrust, true);
	estimateDisparity(tempVertical, tempVerticalTrust, false);


	DisparityMap disparity(lightField, "final disparity");
	Map1d trust(lightField->viewHeight(), lightField->viewWidth(), "final disparity trust");
	mergeDisparities(tempHorizontal, tempVertical,tempHorizontalTrust,tempVerticalTrust,disparity, trust);
	return disparity;
}

const void StructureTensorEstimator::mergeDisparities(DisparityMap& horizontalDisp,
													  DisparityMap& verticalDisp, 
												      Map1d& horizontalTrust, 
													  Map1d& verticalTrust,
													  DisparityMap& finalDisp,
													  Map1d& finalTrust) {
	for (int n = 0; n < finalDisp.rows(); ++n) {
		for (int m = 0; m < finalDisp.cols(); ++m) {

			double trustHoriz = horizontalTrust(n, m);
			double trustVert = verticalTrust(n, m);
			if (trustHoriz >= trustVert) {
				finalDisp(n, m) = horizontalDisp(n, m);
				finalTrust(n, m) = horizontalTrust(n, m);
			}
			else {
				finalDisp(n, m) = verticalDisp(n, m);
				finalTrust(n, m) = horizontalTrust(n, m);
			}
		}
	}
	std::cout << "BeforeInpaintMSE = " << finalDisp.mse(15) << std::endl;
	try {
		Mat mask = finalDisp < 1.5 * lightField->getDispRange()[0] | finalDisp > 1.5 * lightField->getDispRange()[1];
		Mat floatDisp;
		finalDisp.data.convertTo(floatDisp, CV_32F);
	
		cv::inpaint(floatDisp, mask, floatDisp, 3, cv::INPAINT_TELEA);
		floatDisp.convertTo(finalDisp.data, CV_64F);
	}
	catch (cv::Exception& e) {
		std::cout << "Error Estimating Initial Disparity: " << e.what() << std::endl;
		exit(-1);
	}
	std::cout << "afterInpaintMSE = " << finalDisp.mse(15) << std::endl;
	try {
		finalDisp.data = max(min(finalDisp, lightField->getDispRange()[1]), lightField->getDispRange()[0]);
	}
	catch (cv::Exception& e) {
		std::cout << "Error Estimating Initial Disparity: " << e.what() << std::endl;
		exit(-1);
	}
	finalDisp.show();

	std::cout << "afterRAMSE = " << finalDisp.mse(15) << std::endl;

}

const void StructureTensorEstimator::estimateDisparity(DisparityMap& dispOut,Map1d& trustOut, bool isHorizontal) {
	int epiWidth;
	int epiHeight;
	int sizeFixedPosition;
	int lr = this->lightField->viewMatrixHeight() / 2;
	int kr = this->lightField->viewMatrixWidth() / 2;
	int fixedView;
	if (isHorizontal) {
		//only sampling crosshair views
		fixedView = lr;
		epiWidth = this->lightField->viewWidth();
		epiHeight = this->lightField->viewMatrixWidth();
		sizeFixedPosition = this->lightField->viewHeight();
	}
	else {
		fixedView = kr;
		epiWidth = this->lightField->viewHeight();
		epiHeight = this->lightField->viewMatrixHeight();
		sizeFixedPosition = this->lightField->viewWidth();
	}

	for (uint i = 0; i < sizeFixedPosition; i++) {


		image::Image epi = this->lightField->getEpi(fixedView, i, isHorizontal);
		Mat disparity = cv::Mat(epiHeight - 2, epiWidth, CV_64F);
		Mat trust = cv::Mat(epiHeight - 2, epiWidth, CV_64F);
		
		disparityFromEPI(epi, disparity, trust);
		
		for (uint j = 0; j < epiWidth; j++) {
			double d = *disparity.ptr<double>(fixedView-1, j);
			double r = *trust.ptr<double>(fixedView-1, j);
			if (!isfinite(d)) d = -2500;
			if (!isfinite(r)) r = 0;
			if (isHorizontal) {
				dispOut(i, j) = d;
				trustOut(i, j) = r;
			}
			else {
				dispOut(j, i) = d;
				trustOut(j, i) = r;
			}
			
			//
		}
		std::cout << "line: " << i << "		\r";
	}

}

const void StructureTensorEstimator::disparityFromEPI(image::Image& inputEPI, cv::Mat& disparityOut, cv::Mat& trustOut) {
	// Normalize EPI so each value is between 0 and 1
	cv::Mat img;
	inputEPI.data.convertTo(img, CV_32F);
	img /= 255;
	

	cv::Mat disparity, trust;
	

	//cout<<disparityOut.cols << " " << disparityOut.rows << endl;

	//Get A Difference Kernel Matrix
	DiffKernelType diffKernelType = DiffKernelType::Gaussian;
	MatrixXd kernelEigenX = diffKernelType.getMatrix(inputEPI.rows() - 2, inputEPI.rows() - 2, false, false, outerScale, outerScale);
	MatrixXd kernelEigenY = kernelEigenX.transpose();
	//Convert Eigen Matrix to OpenCV Matrix
	Mat kernelX;
	Mat kernelY;
	eigen2cv(kernelEigenX, kernelX);
	eigen2cv(kernelEigenY, kernelY);
	
	//Calc derivative estimate using the difference of gaussians kernel
	Mat imgDiffX, imgDiffY;
	try {
		filter2D(img, imgDiffX, -1, kernelX, cv::Point(-1, -1), 0.0, BORDER_REPLICATE);
		filter2D(img, imgDiffY, -1, kernelY, cv::Point(-1, -1), 0.0, BORDER_REPLICATE);
	}
	catch (cv::Exception& e) {
		std::cout << "Error Estimating Initial Disparity: " << e.what() << std::endl;
		std::cout << std::endl;
	}
	//Remove the first and last rows of the image as they're usually innacurate and contribute to loss of accuracy in the final filtering;
	imgDiffX = imgDiffX(Range(1, inputEPI.rows() - 1), Range(0, inputEPI.cols()));
	imgDiffY = imgDiffY(Range(1, inputEPI.rows() - 1), Range(0, inputEPI.cols()));
	//std::cout << imgDiffX.at<float>(3,30) << std::endl;
	
	//Compute components of the structure tensor for each pixel of the EPI
	Mat imgDiffXX, imgDiffYY, imgDiffXY;
	multiply(imgDiffX, imgDiffY, imgDiffXY);

	multiply(imgDiffX, imgDiffX, imgDiffXX);
	multiply(imgDiffY, imgDiffY, imgDiffYY);

	//Final filtering operation for each component.
	Mat J11, J22, J12;      // J11, J22 and J12 are GST components
	GaussianBlur(imgDiffXX, J11, Size(inputEPI.rows() - 2, inputEPI.rows() - 2), innerScale, innerScale, BORDER_REPLICATE);
	GaussianBlur(imgDiffYY, J22, Size(inputEPI.rows() - 2, inputEPI.rows() - 2), innerScale, innerScale, BORDER_REPLICATE);
	GaussianBlur(imgDiffXY, J12, Size(inputEPI.rows() - 2, inputEPI.rows() - 2), innerScale, innerScale, BORDER_REPLICATE);

	//Calc disparity as (J22-J11)^2 + SQRT(J22*J11 + 4*J12^2) / 2*J12
	Mat tmp = J22 - J11;
	Mat tmp2;
	sqrt(tmp.mul(tmp) + 4 * J12.mul(J12), tmp2);
	Mat deltaX = tmp + tmp2;
	Mat deltaS = 2 * J12;
	disparity = deltaX / deltaS;
	
	// Changes each NaN value to -2500.
	cv::patchNaNs(disparity, -2500);

	//Calcs the trust for each pixel of the EPI
	Mat D = tmp.mul(tmp) + 4 * J12.mul(J12);		
	Mat d = (J11 + J22).mul(J11 + J22);
	//this is more reliable than patching nans.
	Mat mask = d < 1e-19;
	trust = D / d;

	if (img.channels() == 3) {
	//Splits into the 3 color channels;
	Mat disp[3];
	split(disparity, disp);
	
		float dr, dg, db;
		float tr, tg, tb;
		//iterates over each EPI position
		for (int i = 0; i < disparityOut.rows; ++i)
		{
			cv::Vec3f* disparityPixel = disparity.ptr<cv::Vec3f>(i); // point to first pixel in row
			cv::Vec3f* trustPixel = trust.ptr<cv::Vec3f>(i); // point to first pixel in row
			cv::Vec3b* actuallyZero = mask.ptr<cv::Vec3b>(i); // point to first pixel in row
			
			double* d = disparityOut.ptr<double>(i);
			double* t = trustOut.ptr<double>(i);
			for (int j = 0; j < disparityOut.cols; ++j){
				//Computes trust for a given pixel and color channel
				//If the quotient of the trust was close to zero, sets the trust to zero, else the trust is set to the computed trust.
				dr = disparityPixel[j][2];
				if (actuallyZero[j][2] == 0) {
					tr = trustPixel[j][2];
				}
				else {
					tr = 0;
				}

				dg = disparityPixel[j][1];
				if (actuallyZero[j][1] == 0) {
					tg = trustPixel[j][1];
				}
				else {
					tg = 0;
				}
				db = disparityPixel[j][0];
				if (actuallyZero[j][0] == 0) {
					tb = trustPixel[j][0];
				}
				else {
					tb = 0;
				}
				// Sets the final disparities and trust values as those for the color channel with higher trust.
				if (tr > tg) {
					if (tr > tb) {
						*(d + j) = dr;
						*(t + j) = tr;
					}
					else {
						*(d + j) = db;
						*(t + j) = tb;
					}
				}
				else {
					if (tg > tb) {
						*(d + j) = dg;
						*(t + j) = tg;
					}
					else {
						*(d + j) = db;
						*(t + j) = tb;
					}
				}
			}
		}
	}
	else {
		for (int i = 0; i < disparityOut.rows; ++i)
		{

			float* trustPixel = trust.ptr<float>(i); // point to first pixel in row
			uchar* actuallyZero = mask.ptr<uchar>(i); // point to first pixel in row

			for (int j = 0; j < disparityOut.cols; ++j)
			{

				if (actuallyZero[j] > 0) {
					trustPixel[j] = 0;
				}
			}
		}
		disparity.convertTo(disparity, CV_64F);

		trust.convertTo(trust, CV_64F);
		disparityOut = disparity;
		trustOut = trust;
	}
}
