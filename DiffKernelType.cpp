#include "DiffKernelType.h"
#include "MathUtils.h"
#include <iostream>

using namespace cv;
using namespace Eigen;

static auto pi = acos(-1);
using namespace std;
std::string DiffKernelType::toString() {
	switch (*this) {
	case DiffKernelType::Gaussian:
		return "Gaussian";
	case DiffKernelType::Scharr:
		return "Scharr";
	case DiffKernelType::Sobel:
		return "Sobel";
	case DiffKernelType::ScharrOuter:
		return "ScharrOuter";
	case DiffKernelType::SimpleDiff:
		return "SimpleDiff";
	}
}

DiffKernelType::DiffKernelType(std::string diffKernelString) {
	if (!diffKernelString.compare("Gaussian")) {
		*this = DiffKernelType::Gaussian;
	}
	else {
		if (!diffKernelString.compare("Scharr")) {
			*this = DiffKernelType::Scharr;
		}
		else {
			if (!diffKernelString.compare("Sobel")) {
				*this = DiffKernelType::Sobel;
			}
			else {
				if (!diffKernelString.compare("ScharrOuter")) {
					*this = DiffKernelType::ScharrOuter;
				}
				else {
					if (!diffKernelString.compare("SimpleDiff")){
						*this = DiffKernelType::SimpleDiff;
					}
					else {
						*this = DiffKernelType::SimpleDiff;
					}
				}

			}
		}
	}
}
MatrixXd  DiffKernelType::getMatrix(int height, int width, bool topHalfAverage, bool half, double sigmaX, double sigmaY) {


	MatrixXd kernel;
	switch (*this) {
	case DiffKernelType::Gaussian:
		kernel = getGaussianDerivativeMatrix(height, width, topHalfAverage, half, sigmaX, sigmaY);
		break;
	case DiffKernelType::SimpleDiff:
		kernel = getSimpleDiff();
		break;
	case DiffKernelType::Scharr:
		kernel = getScharr(height, width, topHalfAverage, half);
		break;
	case DiffKernelType::ScharrOuter:
		kernel = getScharrOuter(height, width, topHalfAverage, half);
		break;
	case DiffKernelType::Sobel:
		kernel = getSobel(height, width, topHalfAverage, half);
		break;
	default:
		kernel = getScharrOuter(height, width, topHalfAverage, half);
		break;
	}
	MatrixXd originalKernel = kernel;
	double n = kernel.maxCoeff();

	if (kernel(0, 0) > 9) {
		cout << kernel.sum() << endl;
		cout << endl << endl << kernel << endl << endl;
		cout << endl << endl << originalKernel << endl << endl;
		cout << originalKernel.sum() << endl;
		cout << endl;
		cout << n << endl;
		cout << endl;
	}
	return kernel;
}

MatrixXd DiffKernelType::getSimpleDiff() {
	MatrixXd kernel = Vector3d(1, 0, 1);
	return kernel;
}
MatrixXd DiffKernelType::getScharr(int height, int width, bool top, bool half) {
	ArrayXd j_start = ArrayXd::LinSpaced(height, 0, height - 1) - height / 2;

	ArrayXXd j = j_start.replicate(1, width);
	ArrayXd i_start = ArrayXd::LinSpaced(width, 0, width - 1) - width / 2;
	ArrayXXd i = i_start.replicate(1, height).transpose();

	MatrixXd kernel = i.sign() * 3 / (pow(3, abs(j) - 1)) * 2 / pow(2, abs(i));

	if (half) {
		splitKernel(kernel, top);
	}
	return kernel;
}

MatrixXd DiffKernelType::getScharrOuter(int height, int width, bool top, bool half) {
	ArrayXd j_start = ArrayXd::LinSpaced(height, 0, height - 1) - height / 2;
	ArrayXXd j = j_start.replicate(1, width);
	ArrayXd i_start = ArrayXd::LinSpaced(width, 0, width - 1) - width / 2;
	ArrayXXd i = i_start.replicate(1, height).transpose();

	MatrixXd kernel = i.sign() * 3 / (pow(3, abs(j) - 1)) * pow(2, abs(i)) / pow(2, height / 2);
	if (half) {
		splitKernel(kernel, top);
	}
	return kernel;
}

MatrixXd DiffKernelType::getSobel(int height, int width, bool top, bool half) {
	ArrayXd j_start = ArrayXd::LinSpaced(height, 0, height - 1) - height / 2;
	ArrayXXd j = j_start.replicate(1, width);
	ArrayXd i_start = ArrayXd::LinSpaced(width, 0, width - 1) - width / 2;
	ArrayXXd i = i_start.replicate(1, height).transpose();


	MatrixXd kernel = -i / (i * i + j * j);
	kernel(height / 2, width / 2) = 0;

	if (half) {
		splitKernel(kernel, top);
	}
	return kernel;

}
MatrixXd DiffKernelType::getGaussianDerivativeMatrix(int height, int width, double sigmaX, double sigmaY) {
	ArrayXd x = ArrayXd::LinSpaced(width, 0, width - 1) - width / 2;
	ArrayXd y = ArrayXd::LinSpaced(height, 0, height - 1) - height / 2;
	ArrayXd G = gaussian(y, 0, sigmaY);
	ArrayXd dG = gaussianDerivative(x, 0, sigmaX);
	MatrixXd kernel;
	kernel = G.matrix() * dG.matrix().transpose();
	if (abs(kernel.sum()) > 1e-4) {
		cout << endl << kernel << endl;
		cout << endl << endl;
	}
	return kernel;
}
MatrixXd DiffKernelType::getGaussianDerivativeMatrix(int height, int width, bool top, bool half, double sigmaX, double sigmaY) {
	MatrixXd kernel = getGaussianDerivativeMatrix(height, width, sigmaX, sigmaY);

	if (half) {
		splitKernel(kernel, top);
	}
	return kernel;
}
void DiffKernelType::splitKernel(MatrixXd& kernel, bool top) {
	int middle = (int)kernel.rows() / 2;
	MatrixXd newKernel;
	if (top) {
		newKernel = kernel(seq(0, middle), all);
	}
	else {
		newKernel = kernel(seq(middle, last), all);
	}
	if (abs(kernel.sum()) > 1e-4) {
		cout << " Splitting Not Working?" << endl;
		cout << endl << kernel << endl;
		cout << endl << endl;
		cout << newKernel << endl;
		cout << endl;
	}
	kernel = newKernel;
}
ArrayXd DiffKernelType::gaussian(ArrayXd x, double mean, double std) {
	double factor = 1 / (sqrt(2.0 * pi) * std * std);

	ArrayXd G = exp(-((x - mean) * (x - mean) / (2.0 * std * std))) * factor;
	return G;
}

ArrayXd DiffKernelType::gaussianDerivative(ArrayXd x, double mean, double std) {
	double factor = 1 / (sqrt(2. * pi) * std * std * std);
	ArrayXd G = (x - mean) * exp(-(x - mean) * (x - mean) / (2 * std * std)) * factor;
	return G;
}

