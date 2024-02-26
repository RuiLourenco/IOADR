#pragma once

//
//
#pragma once
#include "Parameters.h"
#include <fstream>
#include <sstream>
#include <string>
#include "json.h"
#include <filesystem>
#include <fstream>
#include <string>
#include "IterativeDisparityImprovement.h"
#include "LightField.h"


using json = nlohmann::json;
//#include <iostream>
//#include "LightField.h"
//#include "DisparityMap.h"
//#include <opencv2/core.hpp>
//#include <opencv2/imgcodecs.hpp>
//#include <opencv2/highgui.hpp>
//#include <vector>
//
//
namespace toadr {


	
	//	struct IterativeSettingsTest {
	//		vector<uint> nTemps = { 4 };
	//		vector<uint> nRegTemps = { 2 };
	//		vector < uint> nIters = { 10 };
	//		vector<double> T0 = { 1 };
	//		vector<double> alpha = { 0.2 };
	//		vector<double> sigma = { 0.02 };
	//		vector<double> lambda = { 0 };
	//		vector<double> gamma = { 0.05 };
	//		vector<int> gammaEvo = { 0 };
	//		vector<double> stdKernel = { 1. };
	//		vector<int> averageWindowSize = { 5 };
	//		vector<int> estimationWindowSize = { 5 };
	//		vector<bool> occlusionAware = { true };
	//	};
	//	struct CostVolumeSettingsTest {
	//		vector < uint> nLabels = { 90 };
	//	};
	//	struct MethodsTest {
	//		vector<CorrespondenceCost> correspondenceCost = { CorrespondenceCost::Variance };
	//		vector<DepthMethod> depthMethods = { DepthMethod::iterative };
	//	};
	//
	//
	//
	//	struct TestingParameters {
	//		IterativeSettingsTest iterSettings = {};
	//		CostVolumeSettingsTest basicSettings = {};
	//		MethodsTest methodsSettings = {};
	//
	//		TestingParameters(MethodsTest methodsSettings, IterativeSettingsTest iterSettings) :
	//			methodsSettings(methodsSettings),
	//			iterSettings(iterSettings) {}
	//		TestingParameters(MethodsTest methodsSettings, CostVolumeSettingsTest basicSettings) :
	//			methodsSettings(methodsSettings),
	//			basicSettings(basicSettings) {}
	//		TestingParameters(MethodsTest methodsSettings, CostVolumeSettingsTest basicSettings, IterativeSettingsTest iterSettings) :
	//			methodsSettings(methodsSettings),
	//			iterSettings(iterSettings),
	//			basicSettings(basicSettings) {}
	//	};
	//
	class TestOADR {

		std::string testDirectory;
		idi::Parameters parameters;
		idi::Settings settings;
		std::ofstream resultCSV;
		
		std::string parameterHeader;
		std::vector<std::string> lightFields;
		std::string lightFieldDataset;
		void writeCSVHeader();
		void writeCSVBody(std::vector<dmap::Results> results);
		void writeCSVBody(dmap::Results result);
		void writeCSV(std::vector<dmap::Results> results);
		idi::Parameters parseParameters(json configFile);
		idi::Settings parseSettings(json configFile);
		void parseLightFields(json configFile);
	public:
		TestOADR(std::string pathCofigFile,std::string pathTestDirectory);
		void run();
		void gridSearch(std::string pathGridSearchFile);
		//void testForMultipleSettingsAndMultipleLightFields(std::vector<std::string> lightFieldPaths, std::vector<idi::CDCParameters> parameters);
		dmap::Results testIterativeOcclusionAwareOptimizer(std::string lightFieldPath, idi::CDCParameters parameters);
	};

}