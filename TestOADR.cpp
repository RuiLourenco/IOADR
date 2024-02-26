
//
//#include <sstream>
#include <filesystem>
#include "LightField.h"
#include "DisparityMap.h"
#include "TestOADR.h"
#include "DiffKernelType.h"
#include <chrono>
//
//
//
//
using namespace toadr;
namespace fs = std::filesystem;//using namespace depthMap;
using namespace std;
using namespace idi;
using namespace lf;

 

TestOADR::TestOADR(std::string pathConfigFile, std::string pathTestDirectory) :
	testDirectory(pathTestDirectory) {

	filesystem::path octavium(testDirectory);
	cout << octavium << endl;
	try {
		fs::create_directory(this->testDirectory);
	}
	catch (const std::exception& ex) {
		std::cerr << ex.what() << std::endl;
		exit(-1);
	}
	resultCSV = ofstream(testDirectory + "/ObjectiveResults.txt", fstream::app);
	std::ifstream configFile(pathConfigFile);
	json configFileJson;
	try {
		configFileJson = json::parse(configFile, nullptr, true, true);
	}
	catch (const std::exception& ex) {
		std::cerr << ex.what() << std::endl;
	}
	parseLightFields(configFileJson);
	parameters = parseParameters(configFileJson);
	settings = parseSettings(configFileJson);
}
void TestOADR::run() {
	for (auto& lfPath : lightFields) {
		LightField lf(lfPath,lightFieldDataset);
		dmap::DisparityMap dm = dmap::DisparityMap::fromIOADR(lf, parameters,settings);
		dmap::Results result = dm.getResults();
		dm.saveAsImage(testDirectory + "/" + lf.name + ".png");
	    dm.saveDiffAsImage(testDirectory + "/" + lf.name + "_Diff" + ".png");
		dm.estimateNormals().save(testDirectory + "/" + lf.name + "_Normals" + ".png");
		//dm.save(testDirectory + "/" + lf.name + "_Normals" + ".png");
//	depthMap.save2PFM(testDirectory + "/" + lf.name + "_" + settings.toFileName() + "_Normals" + ".pfm");
		resultCSV <<lf.name<<","<<result.toCSV() << endl;
	}
}
void TestOADR::gridSearch(std::string gridSearchPath) {

	std::ofstream gridSearchResults = ofstream(testDirectory + "\\GridSearchResults.txt", fstream::app);


	std::ifstream gridSearchFile(gridSearchPath);
	json gridSearch;
	try {
		gridSearch = json::parse(gridSearchFile, nullptr, true, true);
	}
	catch (const std::exception& ex) {
		std::cerr << ex.what() << std::endl;
	}
	auto gridSearchParameters = gridSearch["Parameters"];
	std::vector<idi::Parameters> parameterVec;
	std::vector<std::string> parameterStrings;
	parameterVec.push_back(parameters);
	std::string tmp;
	parameterStrings.push_back(tmp);
	std::string searchParameters;
	for (auto it = gridSearchParameters.begin(); it != gridSearchParameters.end(); ++it)
	{	
		if(!searchParameters.empty()) searchParameters += ",";
		searchParameters += it.key();
		
		int currentSize = parameterVec.size();
		std::vector<idi::Parameters> temp;
		std::vector<std::string> tempStrings;
		cout << it.key() << endl;
		for (int i = 0; i < currentSize;++i) {
			idi::Parameters currPar = parameterVec[i];
			std::string currString = parameterStrings[i];
			int count = 0;
			for (auto v : it.value()) {
				idi::Parameters currParameter = currPar;
				std::string parameterString = currString;
				if (!parameterString.empty()) parameterString += ",";
				parameterString += to_string(v);
				currParameter.changeValueFromString(it.key(), v);

				temp.push_back(currParameter);
				tempStrings.push_back(parameterString);
				count++;
			}			
		}
		parameterVec = temp;	
		parameterStrings = tempStrings;
		
	}
	gridSearchResults << "LF," <<"#,"<<searchParameters<< ","<<dmap::Results::csvHeader() << endl;


	
	
		
	for (auto& lfPath : lightFields) {
		int count = 0;
		LightField lf(lfPath,lightFieldDataset);
		for (auto parameters : parameterVec) {
			dmap::DisparityMap dm = dmap::DisparityMap::fromIOADR(lf, parameters);
			dmap::Results result = dm.getResults();
			dm.saveAsImage(testDirectory + "/" + lf.name +"_"+to_string(count)+ ".png");
			dm.saveDiffAsImage(testDirectory + "/" + lf.name + "_" + to_string(count) + "_Diff" + ".png");
			dm.estimateNormals().save(testDirectory + "/" + lf.name + "_" + to_string(count) + "_Normals" + ".png");
			//dm.save(testDirectory + "/" + lf.name + "_Normals" + ".png");
	//	depthMap.save2PFM(testDirectory + "/" + lf.name + "_" + settings.toFileName() + "_Normals" + ".pfm");
			gridSearchResults << lf.name << "," << to_string(count) << "," <<parameterStrings[count]<<","<< result.toCSV() << endl;
			count++;
		}
	}
	

}
idi::Parameters TestOADR::parseParameters(json configFile) {
	auto parametersJson = configFile["Parameters"];
	idi::Parameters parameters;
	for (auto parameter : idi::Parameters::parameterList()) {
		//cout << parameter << endl;
	}
	for (auto& parameter : idi::Parameters::parameterList()) {
		//cout << parameter << " " << parametersJson[parameter] << endl;
		parameters.changeValueFromString(parameter, parametersJson[parameter]);
		
	}
	return parameters;
}
idi::Settings TestOADR::parseSettings(json configFile) {
	auto settingsJson = configFile["Settings"];
	idi::Settings settings;
	cout << "Hello" << endl;
	for (auto& setting : idi::Settings::settingList()) {
		cout << setting <<endl;
		bool value = settingsJson[setting];
		cout << setting << " = " << value << endl;
		settings.changeValueFromString(setting, value);

	}
	return settings;
}
void TestOADR::parseLightFields(json configFile) {
	auto lightFieldsJson = configFile["lightFields"];
	vector<std::string> LfPaths = lightFieldsJson["paths"];
	lightFields = LfPaths;
	lightFieldDataset = lightFieldsJson["dataset"];
}

void TestOADR::writeCSVHeader() {
	resultCSV <<this->parameterHeader<< "," << dmap::Results::csvHeader() << endl;
}
void TestOADR::writeCSVBody(vector<dmap::Results> results) {
	for (auto result : results) {
		resultCSV << result.toCSV() << endl;
	}
}
void TestOADR::writeCSVBody(dmap::Results result) {
	resultCSV << result.toCSV() << endl;
}
void TestOADR::writeCSV(vector<dmap::Results> results) {
	writeCSVHeader();
	writeCSVBody(results);
}

