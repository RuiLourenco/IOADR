# Iterative Occlusion Aware Depth Refinement (IOADR)

The IOADR algorithm is an accurate local optimization algorithm for disparity estimation from light field images.
A geometric model of light field images is enforced through a novel energy minimization framework that uses several heuristics to obtain likely candidate disparities.

## Results

The test results, including more images than those available in the published article are available in the Tests folder.

### Occlusion Awareness
The difference to ground truth images (a whiter value implies a larger error) show the clear improvements from the Occlusion Aware Algorithm:  

**Unaware**   
![Unaware](https://github.com/RuiLourenco/IOADR/blob/master/Tests/HCI-Training_OcclusionUnaware/cotton_Diff.png)

**Aware**  
![Aware](https://github.com/RuiLourenco/IOADR/blob/master/Tests/HCI-Training_OcclusionAware/cotton_Diff.png)

## Requirements
- OpenCV3 
- Eigen 3.4 

## Building
To build the project in Microsoft Visual Studio, set up the solution with the correct links to OpenCV3 and Eigen 3.4 in your Windows installation. Release mode compilation is recommended for faster run-time.

## Running the program
Run the created executable from a Power Shell.
Example:
     start ".\Iterative Occlusion Aware Depth Refinement.exe" -ArgumentList '-c ../../config.json -n example'



### All Options:

- -c <config_file>          Specify configuration file  
- -gs <grid_search_config>  Enable grid search and specify grid search configuration file  
- -n <test_name>  Provides the name for the test folder  
- -h                        Displays a help message  
