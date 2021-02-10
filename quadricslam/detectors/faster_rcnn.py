# import common
import numpy as np
import sys, os, json, random
import code
import cv2
import matplotlib.pyplot as plt
import seaborn as sns

# import pytorch
import torch
import torchvision
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
print(torch.__version__, torch.cuda.is_available())

# modify system path so file will work when run directly or as a module
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

# import custom python modules
sys.dont_write_bytecode = True
from base.containers import Trajectory, Quadrics, Detections, ObjectDetection

# import gtsam and extension
import gtsam
import gtsam_quadrics


# import some common detectron2 utilities
import detectron2
from detectron2.utils.logger import setup_logger
setup_logger()
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog, DatasetCatalog
from detectron2.layers import Linear, ShapeSpec, batched_nms, cat, nonzero_tuple
from detectron2.structures import Boxes, Instances
from detectron2.structures.boxes import pairwise_iou

def cv2_imshow(image):
    cv2.imshow('image',image)
    cv2.waitKey(0)


class FasterRCNN(DefaultPredictor):
    """
    Compared to using DefaultPredictor, this class returns the full model predictions.
    """
    def __init__(self, model_zoo_path='COCO-Detection/faster_rcnn_R_50_FPN_1x.yaml', batch_size=10, show_results=False):
        # load config
        cfg = get_cfg()
        cfg.merge_from_file(model_zoo.get_config_file(model_zoo_path))
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5 
        cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url(model_zoo_path)
        super().__init__(cfg)
        self.batch_size = batch_size
        self.show_results = show_results

    def preprocess_inputs(self, original_image):
        """
        Returns:
            inputs (dict):
        """
        if self.input_format == "RGB":
            # whether the model expects BGR inputs or RGB
            original_image = original_image[:, :, ::-1]
        height, width = original_image.shape[:2]
        image = self.aug.get_transform(original_image).apply_image(original_image)
        image = torch.as_tensor(image.astype("float32").transpose(2, 0, 1))
        return {"image": image, "height": height, "width": width}

    def __call__(self, images):
        """
        Args:
            original_image (np.ndarray): an image of shape (H, W, C) (in BGR order).

        Returns:
            predictions (dict):
                the output of the model for one image only.
                See :doc:`/tutorials/models` for details about the format.
        """
        # print('Detecting with {} images!'.format(len(images)))
        with torch.no_grad():  # https://github.com/sphinx-doc/sphinx/issues/4258

            # preprocess images
            inputs = [self.preprocess_inputs(image) for image in images]

            # process in batches
            predictions = []
            n_batches = int(np.ceil(len(inputs)/self.batch_size))
            for batch in range(n_batches):
                batched_inputs = inputs[batch*self.batch_size:batch*self.batch_size+self.batch_size]
                batched_outputs = self.manually_predict(self.model, batched_inputs)
                predictions += batched_outputs

            # remove dictionaries
            predictions = [p['instances'] for p in predictions]

            # show results
            if self.show_results:
                for prediction, image in zip(predictions, images):
                    v = Visualizer(image[:, :, ::-1], MetadataCatalog.get(self.cfg.DATASETS.TRAIN[0]), scale=1.2)
                    out = v.draw_instance_predictions(prediction.to('cpu'))
                    cv2_imshow(out.get_image()[:, :, ::-1])

            # put in QSLAM object
            detections = []
            for prediction in predictions:
                image_detections = []
                for i in range(len(prediction)):
                    instance = prediction[i]
                    b = instance.pred_boxes.tensor.squeeze(0).cpu().numpy()
                    box = gtsam_quadrics.AlignedBox2(b[0], b[1], b[2], b[3]) 
                    distribution = np.zeros(80)
                    distribution[instance.pred_classes.item()] = instance.scores.item()
                    detection = ObjectDetection(box, 1.0, distribution)
                    detection.fc1 = instance.fc1
                    detection.fc2 = instance.fc2
                    image_detections.append(detection)
                detections.append(image_detections)

            return detections


    def manually_predict(self, model, batched_inputs):
        """
        Returns Instances object w/ boxes, scores, pred_classes and fc2
        """
        images = model.preprocess_image(batched_inputs) # moves images to gpu
        features = model.backbone(images.tensor) # p2,p3,p4,p5,p6: 1x256x200x272
        proposals, _ = model.proposal_generator(images, features, None) # Instances(proposal_boxes, objectness_logits)
        # pass through roi_heads and post process
        # results, _ = self.roi_heads(images, features, proposals, None)
        # outputs = model._postprocess(results, batched_inputs, images.image_sizes)

        # attach forward hooks to box_head for fc2
        activation = {}
        def get_activation(name):
            def hook(model, input, output):
                activation[name] = output.detach()
            return hook
        model.roi_heads.box_head.fc2.register_forward_hook(get_activation('fc2'))
        model.roi_heads.box_head.fc1.register_forward_hook(get_activation('fc1'))

        # pass features through roi_heads
        features = [features[f] for f in model.roi_heads.box_in_features]
        box_features = model.roi_heads.box_pooler(features, [x.proposal_boxes for x in proposals])
        box_features = model.roi_heads.box_head(box_features)
        predictions = model.roi_heads.box_predictor(box_features)

        # NOTE: must have fix applied to detectron2
        pred_instances, kept_indices = model.roi_heads.box_predictor.inference(predictions, proposals) 
        outputs = model._postprocess(pred_instances, batched_inputs, images.image_sizes)

        ## pool activation maps using predicted boxes
        # pred_boxes = [x.pred_boxes for x in pred_instances] # [n_instances]*n_images
        # rois = model.roi_heads.box_pooler(features, pred_boxes) # activations for predboxes (total_instances x 256 x 7 x 7)
        # pass through box_head to capture fc1/fc2 of final predictions
        # model.roi_heads.box_heads(rois)
        # code.interact(local=dict(globals(),**locals()))
        
        # extract relevent fc2 features for each kept instance
        # reshape (n_images*1000 x 1024) -> (n x 1000 x 1024)
        n_images = len(batched_inputs)
        activation['fc2'] = activation['fc2'].reshape(n_images, activation['fc2'].shape[0]//n_images, -1)
        activation['fc1'] = activation['fc1'].reshape(n_images, activation['fc1'].shape[0]//n_images, -1)
        # append fc2 to Instances for each image
        for i in range(n_images):
            outputs[i]['instances'].fc2 = activation['fc2'][i, kept_indices[i], :]
            outputs[i]['instances'].fc1 = activation['fc1'][i, kept_indices[i], :]
        return outputs




if __name__ == '__main__':

    test_image = cv2.imread('/home/lachness/test.png')
    test_images = [test_image]*10

    # create predictor
    predictor = FasterRCNN('COCO-Detection/faster_rcnn_R_50_FPN_1x.yaml', batch_size=5)

    # run detector
    predictions = predictor(test_images)

    for image, prediction in zip(test_images, predictions):
        v = Visualizer(image[:, :, ::-1], MetadataCatalog.get(predictor.cfg.DATASETS.TRAIN[0]), scale=1.2)
        out = v.draw_instance_predictions(prediction.to('cpu'))
        # out = v.draw_instance_predictions(detection.to('cpu'))
        cv2_imshow(out.get_image()[:, :, ::-1])

        

    # features:
    # p2: nimg x 256 x 200 x 272
    # p3: nimg x 256 x 100 x 136
    # p4
    # pooled: ninst x 256 x 7 x 7 (12544 vector per instance)