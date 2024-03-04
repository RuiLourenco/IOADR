# Iterative Occlusion Aware Depth Refinement (IOADR)

The IOADR algorithm is an accurate local optimization algorithm for disparity estimation from light field images.
A geometric model of light field images is enforced through a novel energy minimization framework that uses several heuristics to obtain likely candidate disparities.

## Requirements
- OpenCV3 
- Eigen 3.4 

## Building
To build the project in Microsoft Visual Studio, set up the solution with the correct links to OpenCV3 and Eigen 3.4 in your Windows installation. Release mode compilation is recommended for faster run-time.

##Running the program
Run the created executable from a command line.
Example:   

    /"Iterative Occlusion Aware Depth Refinement.exe"/ -c ../exampleConfigs/config.json -n hci_tests"

All Options:

-c <config_file>          Specify configuration file
-gs <grid_search_config>  Enable grid search and specify grid search configuration file
-n <test_name>  Provides the name for the test folder
-h                        Displays a help message

