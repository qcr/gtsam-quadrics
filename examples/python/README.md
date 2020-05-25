# README - Python Examples #

## Basic Examples: ## 

* **simple_example.py**: In this short python script we add a number of poses / quadrics to the initial estimate and add odometry / boundingbox measurements to the factor graph. We generate boundingbox measurements by reprojecting the quadric into the image at each pose and calculating the conics bounds. 

## Example Frontend ##

* **example_frontend/system.py**: The python system can be run on the SceneNetRGBD dataset or a short toy dataset. 
<!-- TODO: describe how to change settings -->

If you wish to run the example python frontend on the SceneNetDataset, we require: 

* [pySceneNetRGBD](https://github.com/jmccormac/pySceneNetRGBD) cloned and build such that scenenet_pb2.py exists in root folder
* [SceneNetRGBD Data](https://robotvault.bitbucket.io/scenenet-rgbd.html) train and/or validation downloaded associated with protobuf files
