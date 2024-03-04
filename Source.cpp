#pragma once
#include <iostream>
#include "LightField.h"
#include "PointProjectionImage.h"
#include "Image.h"
#include "MathUtils.h"
#include "StructureTensorEstimator.h"
#include "IterativeDisparityImprovement.h"
#include <opencv2/core.hpp>
#include "TestOADR.h"

//#include "matplotlibcpp.h"

//namespace plt = matplotlibcpp;

using namespace toadr;
using namespace std;

struct CommandLineArgs {
    string configFile;
    bool doGridSearch;
    string gridSearchConfigFile;
    string testName = "legoTruck-OcclusionUnaware";
};





void displayHelp() {
    cout << "Usage: program_name [options]\n";
    cout << "Options:\n";
    cout << "  -c <config_file>          Specify configuration file\n";
    cout << "  -gs <grid_search_config>  Enable grid search and specify grid search configuration file\n";
    cout << "  -n <test_name>  Provides the name for the test folder\n";
    cout << "  -h                        Display this help message\n";
}

CommandLineArgs parseCommandLine(int argc, char* argv[]) {
    CommandLineArgs args;

    // Set default values
    args.doGridSearch = false;

    // Skip program name (argv[0])
    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];
        if (arg == "-h") {
            displayHelp();
            exit(0);
        }
        else if (arg == "-gs" && i + 1 < argc) {
            args.doGridSearch = true;
            args.gridSearchConfigFile = argv[i + 1];
            i++; // Skip the next argument since it's the grid search config file path
        }
        else if (arg == "-c" && i + 1 < argc) {
            args.configFile = argv[i + 1];
            i++; // Skip the next argument since it's the config file path
        }
        else if (arg == "-n" && i + 1 < argc) {
            args.testName = argv[i + 1];
            i++;
        }
    }

    // If -gs is specified, but grid search config file is missing, display error message
    if (args.doGridSearch && args.gridSearchConfigFile.empty()) {
        cerr << "Error: -gs option requires a grid search configuration file.\n";
        exit(1);
    }
    if (args.configFile.empty()) {
        args.configFile = "./config.json";
    }

    return args;
}

int main(int argc, char** argv){
    CommandLineArgs args = parseCommandLine(argc, argv);
    TestOADR test = TestOADR(args.configFile, "./tests/"+args.testName);

    if (args.doGridSearch) {
        test.gridSearch(args.gridSearchConfigFile);
    }
    else {
        test.run();
    }
	return 1;
}
