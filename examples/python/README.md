# README - Python Examples #

## Basic Examples: ## 

* **simple_example.py**: In this short python script we add a number of poses / quadrics to the initial estimate and add odometry / boundingbox measurements to the factor graph. We generate boundingbox measurements by reprojecting the quadric into the image at each pose and calculating the conics bounds. 

## Example Frontend ##

* **example_frontend/system.py**: The python system can be run on the SceneNetRGBD dataset or a short toy dataset. 
<!-- TODO: describe how to change settings -->

If you wish to run the example python frontend on the SceneNetDataset, we require: 

* [pySceneNetRGBD](https://github.com/jmccormac/pySceneNetRGBD) cloned and built such that scenenet_pb2.py exists in root folder
* [SceneNetRGBD Data](https://robotvault.bitbucket.io/scenenet-rgbd.html) train and/or validation downloaded associated with protobuf files
* [ShapeNet Dataset](https://www.shapenet.org/) used to access 3D bounds of SceneNet objects (required if initializing quadrics from SceneNet dataset)

### Settings ### 

The front-end can be run using `python3 examples/python/example_frontend/system.py --config examples/python/example_frontend/SceneNetConfig.ini`

We have provided config files for use with SceneNet and a simple Simulated dataset. In order to run the system on SceneNet, you will need to modify SceneNetConfig.ini with the correct paths to the dataset. Additionally, you can modify the seed, quadric initialization method, noise levels and noise estimates. A description of these variables can be found below:

* **Seed**: the numpy seed for the run 
* **PRIOR_SIGMA**: the noise estimate for the prior factor on the first pose 
* **ODOM_SIGMA**: the noise estimate for the between factors of odometry measurements 
* **ODOM_NOISE**: the standard deviation of noise to be introduced to the true odometry measurements
* **BOX_SIGMA**: the noise estimate for the boundingbox factors of box measurements, as a standard deviation of pixels. 
* **BOX_NOISE**: the standard deviation of noise to be introduced to the true boundingbox measurements 
* **Initialization**: controls how quadrics are initialized, either 'SVD' to be constructed from noisy measurements, or 'Dataset' to use the true dataset quadrics. 
* **Dataset**: controls which dataset to be run, either 'SceneNet' or 'Simulated'.
