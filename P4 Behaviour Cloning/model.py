# Import required modules
import csv
import cv2
import numpy as np
import os
import sklearn
import math
import matplotlib.pyplot as plt
import random
from scipy import ndimage

# get_data function obtained data from the working directory and returns the training/validation data in an array
def get_data():
    # Used to split up data for testing and validation
    from sklearn.model_selection import train_test_split
    
    # Store each line in the csv as a "sample"
    samples = []
    with open('/root/Desktop/data/driving_log.csv') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            samples.append(row)
    
    # Split the data into 20% for testing/validation, also shuffles the data by default
    train_samples, validation_samples = train_test_split(samples, test_size=0.2)
    
    return train_samples, validation_samples

# plot_hist() takes the samples and plots a histogram with 100 bins and a width of 0.02 for angle
def plot_hist(samples):
    all_data = []
    # Appends only the angle measurement of each sample
    for sample in samples:
        all_data.append(float(sample[3]))
    
    # Create a histogram using numpy
    sample_hist, sample_edges = np.histogram(all_data, bins=100)

    # Plot histograms with a width of 0.02 degree separation
    plt.bar(sample_edges[:-1], sample_hist, width = 0.02)
    plt.title('Training Data')
    plt.xlim(min(sample_edges), max(sample_edges))
    plt.show()

# generator() generates data in batches using yield instead of return
def generator(samples, batch_size=32):
    # Number of data samples in total
    num_samples = len(samples)
    # Infinite loop to continuously generate data using hte shuffled input data
    while 1:
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]
            
            # For each batch, a set of images and angles are stored for all 3 cameras on the simulated car
            images = []
            angles = []
            for batch_sample in batch_samples:
                # tuned parameter for angle correction for left/right cameras
                correction = 0.2 
                
                # Store the center angle value as a float
                steering_center = float(batch_sample[3])
                
                # Apply correction to steering angle for left/right images
                steering_left = steering_center + correction
                steering_right = steering_center - correction

                # Read in images from center, left, and right cameras
                img_center = ndimage.imread(batch_sample[0].strip())
                img_left = ndimage.imread(batch_sample[1].strip())
                img_right = ndimage.imread(batch_sample[2].strip())
                
                # Add to the array until all samples are in
                images.extend((img_center, img_left, img_right))
                angles.extend((steering_center, steering_left, steering_right))
            
            # Created an augmented set that doubles all data by including a flipped version for pre-processing
            augmented_images, augmented_angles = [], []
            for image, angle in zip(images, angles):
                augmented_images.append(image)
                augmented_angles.append(angle)
                # Both the angle and image is flipped (horizontally)
                augmented_images.append(cv2.flip(image,1))
                augmented_angles.append(angle * -1)
            
            # Convereted to an array as the correct input data type
            X_train = np.array(augmented_images)
            y_train = np.array(augmented_angles)
            
            # Yield to resume the next batch when the generator is called again
            yield sklearn.utils.shuffle(X_train, y_train)

# LeNet_gen() uses the LeNet architecture to train the model, this provided superior performance after testing
# The batch size, and two types of generators are inputted
def LeNet_gen(batch_size, train_generator, validation_generator):
    # Import Keras modules
    from keras.models import Sequential
    from keras.layers import Flatten, Dense, Lambda, Cropping2D, Conv2D
    from keras.layers.pooling import MaxPooling2D
    
    # The model architecture is described in layers below
    model = Sequential()
    model.add(Cropping2D(cropping=((70,25),(0,0)), input_shape=(160,320,3)))
    model.add(Lambda(lambda x: x/ 255.0 - 0.5))
    model.add(Conv2D(6,5,5,activation='relu'))
    model.add(MaxPooling2D())
    model.add(Conv2D(6,5,5,activation='relu'))
    model.add(MaxPooling2D())
    model.add(Flatten())
    model.add(Dense(120))
    model.add(Dense(86))
    model.add(Dense(1))
    
    # Model is compiled using the mean-squared error and Adam optimizer
    # The history object contained the loss graph
    model.compile(loss='mse', optimizer='adam')
    history_object = model.fit_generator(train_generator, steps_per_epoch=math.ceil(len(train_samples)/(batch_size)), validation_data=validation_generator, validation_steps=math.ceil(len(validation_samples)/(batch_size)), epochs=7, verbose=1)
    
    # Save model
    model.save('model.h5')
    
    return history_object
        
# nvidia_gen() uses the Nvidia autonomous car architecture to train the model, this was less consistent through testing
# The batch size, and two types of generators are inputted
def nvidia_gen(batch_size, train_generator, validation_generator):
    from keras.models import Sequential
    from keras.layers import Flatten, Dense, Lambda, Cropping2D, Dropout
    from keras.layers.convolutional import Conv2D

    # The model architecture is described in layers below
    model = Sequential()
    model.add(Lambda(lambda x: x / 127.5 - 1.0, input_shape=(160,320,3)))
    model.add(Cropping2D(cropping=((70,25),(0,0))))
    model.add(Conv2D(24,5,5,subsample=(2,2),activation='relu'))
    model.add(Conv2D(36,5,5,subsample=(2,2),activation='relu'))
    model.add(Conv2D(48,5,5,subsample=(2,2),activation='relu'))
    model.add(Conv2D(64,3,3,activation='relu'))
    model.add(Conv2D(64,3,3,activation='relu'))
    model.add(Flatten())
    model.add(Dense(100))
    model.add(Dropout(0.5))
    model.add(Dense(50))
    model.add(Dropout(0.5))
    model.add(Dense(10))
    model.add(Dropout(0.5))
    model.add(Dense(1))
    
    # Model is compiled using the mean-squared error and Adam optimizer
    # The history object contained the loss graph
    model.compile(loss='mse', optimizer='adam')
    history_object = model.fit_generator(train_generator, steps_per_epoch=math.ceil(len(train_samples)/(batch_size)), validation_data=validation_generator, validation_steps=math.ceil(len(validation_samples)/(batch_size)), epochs=7, verbose=1)
    
     # Save model
    model.save('model.h5')

    return history_object

# visualize_loss() uses the history object provided from training hte model to plot the loss
def visualize_loss(history_object):
    # Print the keys contained in the history object
    print(history_object.history.keys())
    
    # Plot the training and validation loss for each epoch
    plt.plot(history_object.history['loss'])
    plt.plot(history_object.history['val_loss'])
    plt.title('Model -  Mean-squared Error Loss')
    plt.ylabel('Mean-squared Error Lossoss')
    plt.xlabel('Epoch')
    plt.legend(['training set', 'validation set'], loc='upper right')
    plt.show()
    

# Main executing functions below
train_samples, validation_samples = get_data()
plot_hist(train_samples)

# Set batch size - but will be multiplied by 6 to generate more data within the generator 
# Includes a normal and flipped version of the left, center, and right images with adjusted steering angles
batch_size = 32

# Use the generator function to create the shuffled data one batch at a time
train_generator = generator(train_samples, batch_size=batch_size)
validation_generator = generator(validation_samples, batch_size=batch_size)

# Train the model using gneerators
history_object = nvidia_gen(batch_size, train_generator, validation_generator)

# Visualize the loss of the trained model
visualize_loss(history_object)