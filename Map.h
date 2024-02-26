#pragma once

#include <opencv2/core.hpp>

namespace map {
	template <class T>
	//A Virtual Class that facilitates openCV Matrix look ups and image visualisation.
	class Map
	{
		public:
			//constructors
			//Map() = default;
			//Map(cv::Mat_<T> disparityMap);

			//getters
			

			/**
			 * @brief Returns reference to the Map at point (n,m).
			 * @param n: The row index.
			 * @param m: The column index.
			 * @return A plane vector
			 */
			T& operator()(uint n, uint m);
			
			/**
			 * @brief Provides implicit cast to a const cv::Mat
			 */
			operator cv::Mat() const;
			/**
			 * @brief Provides implicit cast to cv::Mat&
			 */
			//operator cv::Mat& ();
			operator cv::Mat ();
			T* ptr(uint n, uint m);
			T* ptr(uint r);
			T at(uint n, uint m);
			int cols();
			int rows();
			const bool isInBounds(int n, int m);
			

			cv::Mat_<T> data;
			~Map() = default;	
	};
	template <class T> __forceinline int Map<T>::cols() {
		return this->data.cols;
	}
	template <class T> __forceinline int Map<T>::rows() {
		return this->data.rows;
	}
	template <class T> __forceinline T* Map<T>::ptr(uint n, uint m) {
		return this->data.ptr<T>(n, m);
	}
	template <class T> __forceinline T* Map<T>::ptr(uint r) {
		return this->data.ptr<T>(r);
	}
	template <class T> __forceinline T& Map<T>::operator()(uint n, uint m) {
		return *ptr(n, m);
	}


	template <class T> __forceinline Map<T>::operator cv::Mat() const {
		return data;
	}
	//template <class T> __forceinline Map<T>::operator cv::Mat& () {
	//	return data;
	//}
	template <class T> __forceinline Map<T>::operator cv::Mat () {
		return data;
	}
	template <class T> __forceinline const bool Map<T>::isInBounds(int n, int m) {
		bool inBounds = (n >= 0 && m >= 0 && n < rows() && m < cols());
		return inBounds;
	}



}


