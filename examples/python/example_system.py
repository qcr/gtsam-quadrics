
# def _create_offline_optimizer(self):
#     params = gtsam.LevenbergMarquardtParams()
#     params.setVerbosityLM("SILENT") #SUMMARY
#     params.setMaxIterations(20)
#     params.setlambdaInitial(1.0e-5)
#     params.setlambdaUpperBound(1.0e+10)
#     params.setlambdaLowerBound(1.0e-8)
#     params.setRelativeErrorTol(1.0e-5)
#     params.setAbsoluteErrorTol(1.0e-5)
#     optimizer = gtsam.LevenbergMarquardtOptimizer(self.global_graph, self.global_values, params)
#     self.current_estimate = optimizer.optimize()



class OnlineSystem(object):
    def __init__(self, calibration):
        self.calibration = calibration

        # create isam optimizer
        self.isam = self._create_online_optimizer()

        # shortcut for gtsam variables
        self.X = lambda i: int(gtsam.symbol(ord('x'), i))
        self.Q = lambda i: int(gtsam.symbol(ord('q'), i))

        # declare noise models
        self.odom_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([0.001]*6, dtype=np.float))
        self.box_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([20]*4, dtype=np.float))
        self.pose_prior_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([1.0]*6, dtype=np.float))

    def _create_online_optimizer(self):
        opt_params = gtsam.ISAM2DoglegParams()
        # opt_params = gtsam.ISAM2GaussNewtonParams()
        # opt_params.setWildfireThreshold(1e-5)

        parameters = gtsam.ISAM2Params()
        parameters.setOptimizationParams(opt_params)
        parameters.setEnableRelinearization(True)
        parameters.setRelinearizeThreshold(0.01)
        parameters.setRelinearizeSkip(1)
        parameters.setCacheLinearizedFactors(False)
        isam = gtsam.ISAM2(parameters)
        return isam

    def update(self, odom, boxes)

        # filter object detections
        # image_detections = self.filter_detections(image_detections)

        # run data association
        associated_boxes = self.associate(boxes)

        # store new boxes for later initialization and factor adding
        self.detections.add_detections(associated_boxes)


        # create local graph and estimate 
        graph = gtsam.NonlinearFactorGraph()
        estimate = gtsam.Values()






        # add new pose measurements to values
        local_estimate.insert(self.X(pose_key), camera_pose)

            # add prior for first few poses
            if pose_key < self.config['QuadricSLAM.n_priors']:
                prior_factor = gtsam.PriorFactorPose3(self.X(pose_key), camera_pose, self.pose_prior_noise)
                local_graph.add(prior_factor)

            # add odom factor 
            if pose_key > 0:
                odom = self.prev_pose.between(camera_pose)
                odom_factor = gtsam.BetweenFactorPose3(self.X(pose_key-1), self.X(pose_key), odom, self.odom_noise)
                local_graph.add(odom_factor)
            self.prev_pose = camera_pose

