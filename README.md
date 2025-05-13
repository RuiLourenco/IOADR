# Iterative Occlusion Aware Depth Refinement (IOADR)

The IOADR algorithm is an accurate local optimization algorithm for disparity estimation from light field images.
A geometric model of light field images is enforced through a novel energy minimization framework that uses several heuristics to obtain likely candidate disparities.

## Results

The raw test results, including all generated disparity maps and surface normal maps, are available in the 'tests' folder. The following provides some visual results and a breakdown of obective results for additional datasets.

## Visual Analysis

### Real Light Fields

**LegoTruck**  
![Disparity Map](https://github.com/RuiLourenco/IOADR/blob/master/Tests/legoTruck_OcclusionAware/legoTruck.png)

### Occlusion Awareness
The difference to ground truth images (a whiter value implies a larger error) show the clear improvements from the Occlusion Aware Algorithm:  

**Unaware**   
![Unaware](https://github.com/RuiLourenco/IOADR/blob/master/Tests/HCI-Training_OcclusionUnaware/cotton_Diff.png)

**Aware**  
![Aware](https://github.com/RuiLourenco/IOADR/blob/master/Tests/HCI-Training_OcclusionAware/cotton_Diff.png)

**Unaware**   
![Unaware](https://github.com/RuiLourenco/IOADR/blob/master/Tests/HCI-Training_OcclusionUnaware/dino_Diff.png)

**Aware**  
![Aware](https://github.com/RuiLourenco/IOADR/blob/master/Tests/HCI-Training_OcclusionAware/dino_Diff.png)

### Surface Normal Regularization
The difference surface normal maps reveal the effect of the smooth plane cost and smooth plane heuristic used:

**No Smooth Plane Heuristic**  
![WithoutSmoothPlaneHeuristic](https://github.com/RuiLourenco/IOADR/blob/master/Tests/HCI-Training-NoSmoothPlaneHeuristic/cotton_Normals.png)

**No Smooth Geometry Cost**  
![WithoutSmoothPlaneCost](https://github.com/RuiLourenco/IOADR/blob/master/Tests/HCI-Training_NoSmoothGeometryCost/cotton_Normals.png)

**With Smooth Geometry Cost**  
![WithSmoothPlaneCost](https://github.com/RuiLourenco/IOADR/blob/master/Tests/HCI-Training_OcclusionAware/cotton_Normals.png)

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
| buddha      | 0.350552 | 0.0213663   |
| buddha2     | 0.855003 | 0.116454    |
| horses      | 1.53808  | 0.120076    |
| medieval    | 0.517461 | 0.0601858   |
| monasRoom   | 0.366887 | 0.0723555   |
| papillon    | 0.49296  | 0.131624    |
| stillLife   | 1.01254  | 0.0716229   |

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
