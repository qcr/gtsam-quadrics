



# run on webcam
# python3 webcam.py --id 0 --calibration calib.yaml

# run on dataset
# python3 dataset.py --name scenenet --config conf.yaml

# run on directory
# python3 slam.py --images ../data/images --config conf.yaml


# use in own project (w/wo odom, detector, DA, init)
if __name__ == '__main__':

    
    # create SLAM system
    SLAM = QuadricSLAM(config)

    

    for image in images:

        # ----------------------
        # get odometry
        rpose = SLAM.get_odom(image) 

        # compound for pose
        pose = history.compose(rpose)

        # get detections
        detections = SLAM.get_detections(image)

        # perform DA
        associated_detections = SLAM.associate(detections, image)

        # perform init
        init_quadrics = SLAM.svd(associated_detections)

        # update graph
        SLAM.update(rpose, pose, associated_detections, init_quadrics)

        # optimize
        trajector, quadrics = SLAM.optimize()
        # ----------------------



# WHAT WE PROVIDE

# 1. wrapper to get odometry from some package
# 2. wrapper to get detections from detectors
# 3. simple DA algorithm
# 4. initialization strategies 
# 5. building graph + optimisation


# NOTE: we want to avoid them having to wrap to our objects
# NOTE: someone needs to store all detections for initialisation 
# NOTE: someone needs to store previous pose for odom-traj conversion
# NOTE: can we just use GTSAM values/graph and keys for storage?
# NOTE: QSLAM-user key conversions
    # - either QSLAM stores them internally and reports to user
    # - or user stores them and reports to QSLAM

# PROBLEMS:
# adding odom/pose separetly, must have consistent keys.
# either use says .addpose(x,3) .addodom(o,2,3)
# or they say .add_odom(o) and we do it all


# FUNCTIONS
# add odom to graph
# add pose to values
# add pose&odom to graph&values
# add bbox to graph
# add quadric to values




# NOTE: if they provide odom, use the previous estimate to calc global pose
    # if they provide global pose, find the relative pose with previous estimate



# QSLAM implementation
# add priors on first few poses
# add pose to values
# add odom to graph
# initialize new quadrics
# add boxes to graph if quad initialized
# add quads to values


# NOTE: if offline, build graph, call .optimize()
    #   if online, build local graph then call .optimize()
    
    
    
    
    


class SLAM(object):

    def __init__(self):
        # graph
        # values
        # optimizer

    def add_pose(self, 
                 pose: gtsam.Pose3, 
                 key: int = None
                 ) -> int:
        """Adds pose to values. Uses next avaible key if none provided."""

    def add_odometry(self, 
                     odom: gtsam.Pose3, 
                     rpose: bool = True,
                     start_key: int = None,
                     end_key: int = None,
                     ) -> tuple[int, int]:
        """Adds odometry to graph as BetweenFactor. 
        Extends previous pose if no keys provided."""

    def add_quadric(self,
                    quadric: gtsam_quadrics.ConstrainedDualQuadric,
                    key: int = None
                    ) -> int:
        """Adds quadric to graph."""

    def add_detection(self,
                      box: gtsam_quadrics.AlignedBox2,
                      pose_key: int,
                      object_key: int
                      ):
        """Adds box to graph as BoundingBoxFactor."""

    def optimize(self) -> tuple[dict[int, gtsam.Pose3], dict[int, gtsam_quadrics.ConstrainedDualQuadric]]:
        """Optimize the current graph.
        If system.offline, performs full bundle adjustment.
        If system.online, updates isam2 with the factors and values since last call
        Returns Trajectory, Map.
        """


    # -------------------------------------------------------------------
    # Fast interfaces (useful when using our own odom/detector/DA/init) 
    # -------------------------------------------------------------------

    def update(self, pose, detections): 
        """Update the graph/values with all single-view measurements. 
        Essentially performs DA/init then calls .add_pose(), .add_odom(), .add_quad(), .add_detection()
        """

    def batch_update(self, trajectory, pose_stamped_detections):
        """Updates the graph/values for multi-views."""

    def full_update(self, image): 
        """Calculates odometry, detections, DA, init, then builds graph."""


class Initialize(object):
    def initialize(self, boxes, poses):
        """Attempts to initialize a quadric given """

class DataAssociation(object):
    def associate(self, boxes, trajectory, history, quadrics):
        """Performs data association. Returns associated boxes."""

class OdometryWrapper(object):
    def update(image) -> gtsam.Pose3():
        pass

class Detector(object):
    def detect(image) -> gtsam_quadrics.AlignedBox2:
        pass
    


# Script users:
# User 1: wants to run on webcam
# install dependencies
# clone, cmake, make
# edit ./config/webcam.yaml with camera intrinsics
# python3 examples/webcam.py --id 0 --calibration ./config/webcam.yaml --config ./config/system.yaml
# Output: Draws each webcam frame overlaid with quadrics

# User 2: wants to try a modified error function on SceneNet
# install dependencies, clone
# edit ./gtsam_quadrics/BoundingBoxFactor.cpp with new error function
# cmake, make
# python3 examples/dataset.py --name scenenet --config ./config/system.yaml
# Output: table of trajectory/map errors

# User 3: wants to add quadrics to their SLAM system
# install dependencies, clone
# in their setup:
QSLAM = QuadricSLAM(config)
# in their source update loop:

    # get odometry
    odom = OdometryWrapper.update(image)
    # compound for pose
    pose = history.compose(rpose)

    # get detections
    detections = detector.detect(image)

    # perform DA
    associated_detections = associator.associate()

    # perform init
    quadrics = initializer.initialize()


    # add pose to values
    pose_key = QSLAM.add_pose(pose)

    # add odom to graph
    QSLAM.add_odometry(odom, previous_pose_key, pose_key)

    # add initalized quadrics to values
    for object_key, quadric in quadrics.items():
        QSLAM.add_quadric(quadric, object_key)

    # add new detections to graph
    for detection in associated_detections:
        QSLAM.add_detection(pose_key, detection.object_key, detection.box)
    
    # optimize
    estimated_trajectory, estimated_map = QSLAM.optimize()


# NOTE: if they do their own DA, they will need a datastructure to store box, object_key and pose_key.
# NOTE: if they have to call .add_pose, .add_odometry etc manually, what's the point of using the SLAM system?
    # couldn't they just use gtsam_quadrics and do it themselves?