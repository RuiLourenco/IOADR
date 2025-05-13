# Iterative Occlusion Aware Depth Refinement (IOADR)

The IOADR algorithm is an accurate local optimization algorithm for disparity estimation from light field images.
A geometric model of light field images is enforced through a novel energy minimization framework that uses several heuristics to obtain likely candidate disparities.

## Results

The raw test results, including all generated disparity maps and surface normal maps, are available in the 'tests' folder. The following provides some visual results and a breakdown of obective results for additional datasets.

## Visual Analysis

### Real Light Fields

**LegoTruck**  
<img src="https://github.com/RuiLourenco/IOADR/blob/master/Tests/legoTruck_OcclusionAware/legoTruck.png" alt="legoTruck" width="300"/>

### Occlusion Awareness
The difference to ground truth images (a whiter value implies a larger error) show the clear improvements from the Occlusion Aware Algorithm:  

**Unaware**  
<img src="https://github.com/RuiLourenco/IOADR/blob/master/Tests/HCI-Training_OcclusionUnaware/cotton_Diff.png" width="300">

**Aware**  
<img src="https://github.com/RuiLourenco/IOADR/blob/master/Tests/HCI-Training_OcclusionAware/cotton_Diff.png" width="300">

**Unaware**  
<img src="https://github.com/RuiLourenco/IOADR/blob/master/Tests/HCI-Training_OcclusionUnaware/dino_Diff.png" width="300">

**Aware**  
<img src="https://github.com/RuiLourenco/IOADR/blob/master/Tests/HCI-Training_OcclusionAware/dino_Diff.png" width="300">

### Surface Normal Regularization
The difference surface normal maps reveal the effect of the smooth plane cost and smooth plane heuristic used:

**No Smooth Plane Heuristic**  
<img src="https://github.com/RuiLourenco/IOADR/blob/master/Tests/HCI-Training-NoSmoothPlaneHeuristic/cotton_Normals.png" width="300">

**No Smooth Geometry Cost**  
<img src="https://github.com/RuiLourenco/IOADR/blob/master/Tests/HCI-Training_NoSmoothGeometryCost/cotton_Normals.png" width="300">

**With Smooth Geometry Cost**  
<img src="https://github.com/RuiLourenco/IOADR/blob/master/Tests/HCI-Training_OcclusionAware/cotton_Normals.png" width="300">

### Objective Results for Additional Images

#### HCI Stratified
| Light Field | MSE x100| Badpix 0.07|
|----------|----------|----------|
|  Backgammon  | 3.79512  |3.05%   |
|  Dots  | 6.26033   |  |19.52% |
|  Pyramids  | 0.0410231  | 4.56%   |
|  Stripes  | 0.650726 | 3.02%   |

#### Original HCI Dataset 
| Light Field | MSE x100 | Badpix 0.07 |
|-------------|----------|-------------|
| buddha      | 0.351 | 2.14%   |
| buddha2     | 0.855 | 11.65%    |
| horses      | 1.538  | 12.01%    |
| medieval    | 0.517 |6.02%  |
| monasRoom   | 0.367 | 7.24%   |
| papillon    | 0.493  | 13.17%    |
| stillLife   | 1.013  | 7.17%   |

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
