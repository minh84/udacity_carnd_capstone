from styx_msgs.msg import TrafficLight

import tensorflow as tf
import os
import numpy as np
import cv2

class TLClassifier(object):
    def __init__(self, config):
        self.config = config
        # get model path
        curr_dir = os.path.dirname(os.path.realpath(__file__))
        self.model_path = os.path.join(curr_dir, config['frozen_model_graph'])

        # TODO load classifier
        # load frozen graph
        self.graph = None
        self.session = None
        self.image_tensor = None
        self.detection_boxes = None
        self.detection_scores = None
        self.detection_classes = None
        self.load_model(model_path=self.model_path)

        self.threshold_prob = 0.7
        self.classes_map = {1: TrafficLight.RED,
                            2: TrafficLight.YELLOW,
                            3: TrafficLight.GREEN,
                            4: TrafficLight.UNKNOWN}

    def load_model(self, model_path):
        # load serialized Tensor Graph
        self.graph = tf.Graph()
        with self.graph.as_default():
            graph_def = tf.GraphDef()
            with tf.gfile.GFile(model_path, 'rb') as fid:
                serialized_graph = fid.read()
                graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(graph_def, name='')

        self.session = tf.Session(graph=self.graph)

        # update tensor
        self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = self.graph.get_tensor_by_name('detection_boxes:0')
        self.detection_classes = self.graph.get_tensor_by_name('detection_classes:0')
        self.detection_scores = self.graph.get_tensor_by_name('detection_scores:0')

        # run first prediction with dummy image to initialize (since first run is considerably slow)
        self.init_model()

    def init_model(self):
        # init model by running with null image
        img_w = self.config['camera_info']['image_width']
        img_h = self.config['camera_info']['image_height']

        null_img = np.zeros((img_h, img_w, 3), dtype=np.uint8)
        _ = self.session.run(
            [self.detection_boxes, self.detection_classes, self.detection_scores],
            feed_dict={
                self.image_tensor: np.expand_dims(null_img, axis=0)
            }
        )

    def get_classification(self, cv_image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # TODO implement light color prediction

        # convert from BGR => RGB
        image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # run the detection
        (boxes, classes, scores) = self.session.run(
            [self.detection_boxes, self.detection_classes, self.detection_scores],
            feed_dict = {
                self.image_tensor : np.expand_dims(image_rgb, axis=0)
            }
        )

        classes = np.squeeze(classes).astype(np.int32)
        scores = np.squeeze(scores)

        # we get the one give maximum score
        i_max_score = np.argmax(scores)
        if (scores[i_max_score] > self.threshold_prob):
            return self.classes_map.get(classes[i_max_score], TrafficLight.UNKNOWN)

        return TrafficLight.UNKNOWN
