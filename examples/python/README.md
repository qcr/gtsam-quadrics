# README - Python Examples #

## Basic Examples: ## 

* **simple_example.py**: In this short python script we add a number of poses / quadrics to the initial estimate and add odometry / boundingbox measurements to the factor graph. We generate boundingbox measurements by reprojecting the quadric into the image at each pose and calculating the conics bounds. 

* **realsense.py**: This example uses a realsense camera for rgb+depth, detectron2 for object detections, and opencv2 for odometry. The quadrics are initialized from a single view and are updated incrementally with new measurements, using the GTSAM isam2 optimizer. The odometry and data-assocation are both very poor in this example, so poor performance is expected. 
