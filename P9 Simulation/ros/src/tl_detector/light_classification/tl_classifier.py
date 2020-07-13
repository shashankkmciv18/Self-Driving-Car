import os
#os.environ["CUDA_DEVICE_ORDER"]="PCI_BUS_ID";
#
## The GPU id to use, usually either "0" or "1";
#os.environ["CUDA_VISIBLE_DEVICES"]="1";

import cv2
import numpy as np
from styx_msgs.msg import TrafficLight
from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge
from sklearn.utils import shuffle
from tensorflow.contrib.layers import flatten
import PIL
import yaml
import tensorflow as tf

from tensorflow import ConfigProto
from tensorflow import InteractiveSession

config = ConfigProto()
config.gpu_options.allow_growth = True
session = InteractiveSession(config=config)

class TLClassifier(object):
    def __init__(self): 
        # For Diagnostics
        # Publish cropped TL bounding box
        self.cropped_tl_bb_pub = rospy.Publisher('/cropped_bb', Image, queue_size=1)
        self.bridge = CvBridge()
        self.trainLeNet = False

        self.my_dir = os.path.dirname(os.path.abspath(__file__))

        self.image_width = 32
        self.image_height = 32
        self.image_depth = 3

        self.light2label = {'Red': 0, 'Yellow': 1, 'Green': 2, 'Unknown': 4}
        self.label2light = {0: 'Red', 1: 'Yellow', 2: 'Green', 4: 'Unknown'}
        self.learn_rate = 0.001
        self.BATCH_SIZE = 30
        self.EPOCHS = 4

        self.n_classes = len(self.light2label.keys())

        if self.trainLeNet:
            self.X_train = []
            self.Y_train = []
            
            self.X_valid = []
            self.Y_valid = []
       
            self.X_test = []
            self.Y_test = []

            self.n_total = 0
            self.n_train = 0
            self.n_valid = 0
            self.n_test  = 0


            self.lenetReadTrainData() 

            tf.reset_default_graph()

        self.tf_x = tf.placeholder(tf.float32, (None, self.image_height, self.image_width, self.image_depth))
        self.tf_y = tf.placeholder(tf.int32, (None))
        self.keep_prob = tf.placeholder(tf.float32)
        self.one_hot_y = tf.one_hot(self.tf_y, self.n_classes)

        self.logits = self.LeNet(self.tf_x, self.n_classes, self.keep_prob, image_depth=self.image_depth)
        self.cross_entropy = tf.nn.softmax_cross_entropy_with_logits(labels=self.one_hot_y, logits=self.logits)
        self.loss_operation = tf.reduce_mean(self.cross_entropy)
        self.optimizer = tf.train.AdamOptimizer(learning_rate = self.learn_rate)
        self.training_operation = self.optimizer.minimize(self.loss_operation)
        self.predict_operation = tf.argmax(self.logits, 1)

        self.correct_prediction = tf.equal(tf.argmax(self.logits, 1), tf.argmax(self.one_hot_y, 1))
        self.accuracy_operation = tf.reduce_mean(tf.cast(self.correct_prediction, tf.float32))
        self.saver = tf.train.Saver()
        
        self.model_fname = self.my_dir + '/lenet'

        if self.trainLeNet:
            self.lenetTrain()
        else:
            self.tf_session = tf.Session() 
            self.saver.restore(self.tf_session, self.model_fname)

    def readTrafficLightRGB(self, yamlFile):
        dir_path = os.path.dirname(os.path.realpath(yamlFile))
        images = [] # images
        labels = [] # corresponding labels
        with open(yamlFile) as file:
            light_list = yaml.load(file, Loader=yaml.FullLoader);
            for anno in light_list:
                if anno['class'] == "image":
                    filePath = dir_path + '/' + anno['filename']
                    image = PIL.Image.open(filePath)
                    lights_list = anno['annotations']
                    for light_info in lights_list:
                        label    = light_info['class']
                        y_height = int(light_info['y_height'])
                        x_width  = int(light_info['x_width'])
                        y_min    = int(light_info['ymin'])
                        x_min    = int(light_info['xmin'])
                        box = (x_min, y_min, x_min+x_width, y_min+y_height)
                        roi = image.crop(box)
                        if roi.size[0] > self.image_height or roi.size[1] > self.image_width:
                            roi.thumbnail((self.image_height,self.image_width))
                        data = np.asarray(roi, dtype="uint8")[:,:,:self.image_depth]
                        offset_x = (self.image_width-data.shape[0])//2
                        offset_y = (self.image_height-data.shape[1])//2
                        zero_mat = np.zeros((self.image_height, self.image_width, self.image_depth), dtype = "uint8")
                        zero_mat[offset_x:offset_x+data.shape[0], offset_y:offset_y+data.shape[1]] = data
                        images.append(zero_mat) 
                        labels.append(label)
        file.close()
        return images, labels

    def lenetReadTrainData(self):
        real_images, real_labels = self.readTrafficLightRGB(self.my_dir + '/data/real_training_data/real_data_annotations.yaml')
        sim_images,  sim_labels  = self.readTrafficLightRGB(self.my_dir + '/data/sim_training_data/sim_data_annotations.yaml')

        train_all = real_images + sim_images
        label_all_org = real_labels + sim_labels
        label_all = list(map(lambda x: self.light2label[x], label_all_org))
        
        train_all, label_all = shuffle(train_all, label_all)
        
        self.n_total = len(train_all)
        
        self.n_train = int(float(self.n_total) * 0.7)
        self.n_valid = int(float(self.n_total) * 0.15)
        self.n_test  = self.n_total - self.n_train - self.n_valid
        
        self.X_train = train_all[:self.n_train]
        self.Y_train = label_all[:self.n_train]
        
        self.X_valid = train_all[self.n_train:self.n_train+self.n_valid]
        self.Y_valid = label_all[self.n_train:self.n_train+self.n_valid]
       
        self.X_test = train_all[self.n_train+self.n_valid:]
        self.Y_test = label_all[self.n_train+self.n_valid:]
        
        #self.n_classes = len(np.unique(label_all))
        
        image_shape = train_all[0].shape
        rospy.loginfo("Number of training examples = {0}".format(self.n_train))
        rospy.loginfo("Number of validation examples = {0}".format(self.n_valid))
        rospy.loginfo("Number of testing examples = = {0}".format(self.n_test))
        rospy.loginfo("Image data shape = {0}".format(image_shape))
        rospy.loginfo("Number of classes = {0}".format(self.n_classes))
        rospy.loginfo("Classes: {0}".format(np.unique(label_all_org)))

    def lenetTrain(self):
        with tf.Session() as sess:
            writer = tf.summary.FileWriter('logs', sess.graph)
            sess.run(tf.global_variables_initializer())
            num_examples = len(self.X_train)
            
            rospy.loginfo("Training...")
            rospy.loginfo("")
            train_log = []
            valid_log = []
            for i in range(self.EPOCHS):
                self.X_train, self.Y_train = shuffle(self.X_train, self.Y_train)
                for offset in range(0, num_examples, self.BATCH_SIZE):
                    if offset + self.BATCH_SIZE > num_examples:
                        end = num_examples
                    else: 
                        end = offset + self.BATCH_SIZE
                    batch_x, batch_y = self.X_train[offset:end], self.Y_train[offset:end]
        
                    sess.run(self.training_operation, feed_dict={self.tf_x: batch_x, self.tf_y: batch_y, self.keep_prob: 0.5})
                train_accuracy = self.lenetEvaluate(self.X_train, self.Y_train)
                valid_accuracy = self.lenetEvaluate(self.X_valid, self.Y_valid)
                train_log.append(train_accuracy)
                valid_log.append(valid_accuracy)
                rospy.loginfo("EPOCH {} ...".format(i+1))
                rospy.loginfo("Train = {:.3f}, Valid = {:.3f}".format(train_accuracy, valid_accuracy))
                rospy.loginfo("")
        
            writer.close()
            self.saver.save(sess, self.model_fname)
            rospy.loginfo("Model saved: ".format(self.model_fname))

        with tf.Session() as sess:
            self.saver.restore(sess, self.model_fname)
            test_accuracy = self.lenetEvaluate(self.X_test, self.Y_test)
            rospy.loginfo("Test Accuracy = {:.3f}".format(test_accuracy))


    def lenetEvaluate(self, X_data, Y_data):
        num_examples = len(X_data)
        total_accuracy = 0
        sess = tf.get_default_session()
        for offset in range(0, num_examples, self.BATCH_SIZE):
            if offset + self.BATCH_SIZE > num_examples:
                end = num_examples
            else: 
                end = offset + self.BATCH_SIZE
            batch_x, batch_y = X_data[offset:end], Y_data[offset:end]
            accuracy = sess.run(self.accuracy_operation, feed_dict={self.tf_x: batch_x, self.tf_y: batch_y, self.keep_prob: 1.0})
            total_accuracy += (accuracy * len(batch_x))
    
        return total_accuracy / num_examples
 
    
    def group_norm(self, x, G=32, eps=1e-5, scope='group_norm') :
        with tf.variable_scope(scope) :
            _N, H, W, C = x.get_shape().as_list()
            N = tf.shape(x)[0]
            G = min(G, C)
           
            if C%G != 0:
                raise Exception('Group number must be commensurate with the number of channels in inputs: {}'.format(C))
                
            x = tf.reshape(x, [N, H, W, G, C // G])
            mean, var = tf.nn.moments(x, [1, 2, 4], keep_dims=True)
            x = (x - mean) / tf.sqrt(var + eps)
    
            gamma = tf.get_variable('gamma', [1, 1, 1, C], initializer=tf.constant_initializer(1.0))
            beta = tf.get_variable('beta', [1, 1, 1, C], initializer=tf.constant_initializer(0.0))
    
    
            x = tf.reshape(x, [N, H, W, C]) * gamma + beta
    
        return x
    
    def LeNet(self, x, n_classes, keep_prob, filter_size=5, image_depth=1, filter_depth=16, num_hidden=120):
        # Arguments used for tf.truncated_normal, randomly defines variables for the weights and biases for each layer
        mu = 0
        sigma = 0.1
    
        weights = {
            'conv1_W': tf.Variable(tf.truncated_normal(shape=(filter_size, filter_size, image_depth, filter_depth), mean = mu, stddev = sigma)),
            'conv2_W': tf.Variable(tf.truncated_normal(shape=(filter_size, filter_size, filter_depth, filter_depth), mean = mu, stddev = sigma)),
            'fc1_W': tf.Variable(tf.truncated_normal(shape=(filter_size*filter_size*filter_depth, num_hidden), mean = mu, stddev = sigma)),
            'fc2_W': tf.Variable(tf.truncated_normal(shape=(num_hidden, num_hidden), mean = mu, stddev = sigma)),
            'fc3_W': tf.Variable(tf.truncated_normal(shape=(num_hidden, n_classes), mean = mu, stddev = sigma))
        }
    
        biases = {
            'conv1_b': tf.Variable(tf.zeros(filter_depth)),
            'conv2_b': tf.Variable(tf.zeros(filter_depth)),
            'fc1_b': tf.Variable(tf.zeros(num_hidden)),
            'fc2_b': tf.Variable(tf.zeros(num_hidden)),
            'fc3_b': tf.Variable(tf.zeros(n_classes))
        }
        
        x      = tf.subtract(tf.div(x, 127.5), 1.0)
        conv1  = tf.nn.conv2d(x, weights['conv1_W'], strides=[1, 1, 1, 1], padding='VALID', name='conv1') + biases['conv1_b']
       
        conv1 = self.group_norm(conv1, G=8, scope='group_norm1')
        conv1 = tf.nn.relu(conv1, name='conv1.relu')
        
        conv1 = tf.nn.max_pool(conv1, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='VALID', name='conv1.max_pool')
        
        conv2 = tf.nn.conv2d(conv1, weights['conv2_W'], strides=[1, 1, 1, 1], padding='VALID', name='conv2') + biases['conv2_b']
        
        conv2 = self.group_norm(conv2, G=4, scope='group_norm2')
        conv2 = tf.nn.relu(conv2, name='conv2.relu')
        
        conv2 = tf.nn.max_pool(conv2, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='VALID', name='conv2.max_pool')
        
        fc0   = flatten(conv2)
        
        fc1   = tf.matmul(fc0, weights['fc1_W'], name='matmul1') + biases['fc1_b']
    
        fc1   = tf.nn.relu(fc1, name='relu3')
        fc1   = tf.nn.dropout(fc1, keep_prob=keep_prob, name='dropout1')
      
        fc2   = tf.matmul(fc1, weights['fc2_W'], name='matmul2') + biases['fc2_b']
       
        fc2   = tf.nn.relu(fc2, name='relu4')
    
        fc2   = tf.nn.dropout(fc2, keep_prob=keep_prob, name='dropout2')
        
        logits = tf.matmul(fc2, weights['fc3_W'], name='matmul3') + biases['fc3_b']
        
        return logits
    
    def lenetPredict(self, sess, X_data):
        num_examples = len(X_data)
        result_y = list()
        for offset in range(0, num_examples, self.BATCH_SIZE):
            if offset + self.BATCH_SIZE > num_examples:
                end = num_examples
            else: 
                end = offset + self.BATCH_SIZE
            batch_x = X_data[offset:end]
            batch_y = sess.run(self.predict_operation, feed_dict={self.tf_x: batch_x, self.keep_prob: 1.0})
            result_y.extend(batch_y)
        return result_y

    def detect_light_state(self, image, TL_BB_list):
        pilImage = PIL.Image.fromarray(image)
        images = []
        for bb in TL_BB_list:
            xmin = bb.xmin
            xmax = bb.xmax
            ymin = bb.ymin
            ymax = bb.ymax
            box = (xmin, ymin, xmax, ymax)
            roi = pilImage.crop(box) 
            if roi.size[0] > self.image_height or roi.size[1] > self.image_width:
                roi.thumbnail((self.image_height,self.image_width))
            data = np.asarray(roi, dtype="uint8")[:,:,:self.image_depth]
            offset_x = (self.image_width-data.shape[0])//2
            offset_y = (self.image_height-data.shape[1])//2
            zero_mat = np.zeros((self.image_height, self.image_width, self.image_depth), dtype = "uint8")
            zero_mat[offset_x:offset_x+data.shape[0], offset_y:offset_y+data.shape[1]] = data
            images.append(zero_mat) 

        results = self.lenetPredict(self.tf_session, images)
        return results

    def get_classification(self, image, TL_BB_list, simulator_mode):
        light_states = self.detect_light_state(image, TL_BB_list)
        return max(light_states, key=light_states.count)

                
