# **Traffic Sign Recognition** 

## Writeup

### You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---

**Build a Traffic Sign Recognition Project**

The goals / steps of this project are the following:
* Load the data set (see below for links to the project data set)
* Explore, summarize and visualize the data set
* Design, train and test a model architecture
* Use the model to make predictions on new images
* Analyze the softmax probabilities of the new images
* Summarize the results with a written report


[//]: # (Image References)
[image1]: ./images/data.png "plotting "

[image4]: ./images/im1.png "Traffic Sign 1"
[image5]: ./images/im2.png "Traffic Sign 2"
[image6]: ./images/im3.png "Traffic Sign 3"
[image7]: ./images/im4.png "Traffic Sign 4"
[image8]: ./images/im5.png "Traffic Sign 5"
[image9]: ./images/123.png "Model "
## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/481/view) individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf. You can use this template as a guide for writing the report. The submission includes the project code.

You're reading it! and here is a link to my [project code](https://github.com/shashankkmciv18/Self-Driving-Car/blob/master/P3%20Traffic%20Sign%20Classifier/codes/main.ipynb)


Here is link for hyper parametertuning code[Tuner Code](https://github.com/shashankkmciv18/Self-Driving-Car/blob/master/P3%20Traffic%20Sign%20Classifier/codes/p1.ipynb)

### Data Set Summary & Exploration

#### 1. Provide a basic summary of the data set. In the code, the analysis should be done using python, numpy and/or pandas methods rather than hardcoding results manually.

I used the numpy and other library to calculate summary statistics of the traffic
signs data set:

* The size of training set is  12630 and have images of size (32,32,3) 
* The size of the validation set is  4410 and have images of size (32,32,3)
* The size of test set is  34799 and have images of size (32,32,3)
* The shape of a traffic sign image (32,32,3)
* The number of unique classes/labels 43

#### 2. Include an exploratory visualization of the dataset.

Here is an exploratory visualization of the data set. It is a distplot  showing how the data is distributed.

![alt text][image1]

### Design and Test a Model Architecture

#### 1. Describe how you preprocessed the image data. What techniques were chosen and why did you choose these techniques? Consider including images showing the output of each preprocessing technique. Pre-processing refers to techniques such as converting to grayscale, normalization, etc. (OPTIONAL: As described in the "Stand Out Suggestions" part of the rubric, if you generated additional data for training, describe why you decided to generate additional data, how you generated the data, and provide example images of the additional data. Then describe the characteristics of the augmented training set like number of images in the set, number of images for each class, etc.)

As the first step i have normalised the data , i have choosed minmax scalar over standarisation beacuse as per my experiences it works better in CNNN than that of standarisation.</br>
After that i have applied an CNN as rest of feature extraction will be carrieed inside this CNN only, with help of Various filters.



#### 2. Describe what your final model architecture looks like including model type, layers, layer sizes, connectivity, etc.) Consider including a diagram and/or table describing the final model.

My final model consisted of the following layers:

| Layer         		|     Description	        					| 
| Input           	| Size=32x32x3                                                 	|
|-----------------	|--------------------------------------------------------------	|
| Conv2d(5x5)     	| filters=32,strides=(1,1),padding=valid,<br>activation='relu' 	|
| Conv2d(4x4)     	| filter=80,strides=(1,1),padding='valid'<br>activation='relu' 	|
| MaxPool2d       	| pool_size=(2,2),padding='valid',strides=(2,2)                	|
| Dropout         	| rate=0.23882                                                 	|
| Conv2D(3x3)     	| filter=64,stride=(1,1),padding='valid,<br>activation=relu    	|
| MaxPool2d       	| pool_size=(2,2),padding='valid',strides=(1,1)                	|
| Dropout         	| rate=0.43981                                                 	|
|.                  |.                                                              |
|.                  |.                                                              |
|.                  |.  (please seee main.inpyb file there is full report)          |
|.                  |.                                                              |
|.                  |.                                                              |
|.                  |.                                                              |
| Flatten         	|                                                              	|
| Fully connected 	| units=192,activation=tanh                                    	|
| FullyConnected  	| units=43,activation=softmax                                  	| 


#### 3. Describe how you trained your model. The discussion can include the type of optimizer, the batch size, number of epochs and any hyperparameters such as learning rate.
For this  purpose i always rely on search based hyper parameter tuning which is present in code itself and for a long experiecne with this stuff, one thing i came to know that this way of tuning hyperparameter using GRid Search or Random search is way better than the manual method ,initially i was only getting an accuracy of 89% but after some fine tuning it was able to get 99% of accuracy.</br>
So in this method the machine itself chooses the best optimizer batch size , and other hyper paramters like learning rate.It might take some time but in order to make an robust product we alsways prefer this.</br>
Full detail of the model is in the picture present at bottom.
For training my model i basically made an model initially which gave me 89% result  on validation dataset and afterr fine tuning it with random search  from tensorflow.kerastuner  accuracy of that model became 93% on the validation set without any data augmentation techniques.
Model will give even more accuracy if the dataset will be large, for that there is an technique of data augmentation, which i will do in future in spare time.

#### 4. Describe the approach taken for finding a solution and getting the validation set accuracy to be at least 0.93. Include in the discussion the results on the training, validation and test sets and where in the code these were calculated. Your approach may have been an iterative process, in which case, outline the steps you took to get to the final solution and why you chose those steps. Perhaps your solution involved an already well known implementation or architecture. In this case, discuss why you think the architecture is suitable for the current problem.

My final model results were:
* training set accuracy of 99.995%
* validation set accuracy of 93%
* test set accuracy of 94.5%

If an iterative approach was chosen:
* What was the first architecture that was tried and why was it chosen?
First i have choosen VGG 16 architecture, because it is considered to be an good image classfication alggorithm </br>
but when i was trying to train the model on my computer it take so much of time so i skipped it.
* What were some problems with the initial architecture?
Main problem with the initial architecture was computational limitaion of my no doubt VGG 16 is an good algorithm but </br>
with limited computer processing power training it on local machine might reuire a lot of computational power.
*How was the architecture adjusted and why was it adjusted? Typical adjustments could include choosing a different model architecture, adding or taking away layers (pooling, dropout, convolution, etc), using an activation function or changing the activation function. One common justification for adjusting an architecture would be due to overfitting or underfitting. A high accuracy on the training set but low accuracy on the validation set indicates over fitting; a low accuracy on both sets indicates under fitting.
I removed some of the last layers of VGG 16 and trained it , i.e ib have only taken first 3 COnv2d layers so that it predicts enough good for this project.
* Which parameters were tuned? How were they adjusted and why?
Initially the result was aroung 89% so i tuned whole model using Random SearchCv.

* What are some of the important design choices and why were they chosen? For example, why might a convolution layer work well with this problem? How might a dropout layer help with creating a successful model?
Well Convolutional layer with best hyperparameters bring this model to an state where it is maximum optimised.

### Test a Model on New Images

#### 1. Choose five German traffic signs found on the web and provide them in the report. For each image, discuss what quality or qualities might be difficult to classify.

Here are five German traffic signs that I found on the web:
Here some image are difficult to even to be recognised by human eye like the 3rd one even then the model is able to predict nicely.


![alt text][image4]
</br>
[['Traffic signals', 2.3171337e-17],
 ['Speed limit (50km/h)', 7.392017e-15],
 ['Speed limit (20km/h)', 2.280922e-14],
 ['Speed limit (70km/h)', 9.938535e-14],
 ['Speed limit (30km/h)', 1.0]]
</br>
![alt text][image5]
</br>
[['Speed limit (30km/h)', 1.5306637e-22],
 ['Ahead only', 5.2261523e-22],
 ['Turn left ahead', 6.6438794e-22],
 ['Roundabout mandatory', 7.694055e-21],
 ['Turn right ahead', 1.0]]
</br>
![alt text][image6] 
</br>
[['Speed limit (100km/h)', 5.3222666e-06],
 ['Speed limit (120km/h)', 1.0624367e-05],
 ['Speed limit (60km/h)', 1.3820942e-05],
 ['Speed limit (50km/h)', 0.000772773],
 ['Speed limit (80km/h)', 0.9991961]]
 </br>
![alt text][image7] 
</br>
[['Turn left ahead', 6.584047e-18],
 ['Traffic signals', 6.945318e-16],
 ['Ahead only', 2.4448854e-13],
 ['Keep left', 5.3049947e-13],
 ['Turn right ahead', 1.0]]
 </br>
![alt text][image8]
</br>
[['Right-of-way at the next intersection', 3.9416917e-20],
 ['Bumpy road', 4.1177044e-20],
 ['Keep right', 4.5208295e-19],
 ['Wild animals crossing', 1.1694021e-18],
 ['Road work', 1.0]]
</br>

#### 2. Discuss the model's predictions on these new traffic signs and compare the results to predicting on the test set. At a minimum, discuss what the predictions were, the accuracy on these new predictions, and compare the accuracy to the accuracy on the test set (OPTIONAL: Discuss the results in more detail as described in the "Stand Out Suggestions" part of the rubric).

Here are the results of the prediction:

| Image			        |     Prediction	        					| 
|:---------------------:|:---------------------------------------------:| 
| 30KM/H     		|30KM/h   									| 
| Turn Right ahead     			|Turn Right ahead 										|
| 80Km/h				|80Km/h|
| Turn Right ahead     			|Turn Right ahead 										|
|Road work		|Road Work      							|


The model was able to correctly guess 5 of the 5 traffic signs, which gives an accuracy of 100%. This compares favorably to the accuracy on the test set of ...

#### 3. Describe how certain the model is when predicting on each of the five new images by looking at the softmax probabilities for each prediction. Provide the top 5 softmax probabilities for each image along with the sign type of each probability. (OPTIONAL: as described in the "Stand Out Suggestions" part of the rubric, visualizations can also be provided such as bar charts)

The code for making predictions on my final model is located in the 11th cell of the Ipython notebook.

For the first image, the model is relatively sure that this is a stop sign (probability of 0.6), and the image does contain a stop sign. The top five soft max probabilities were

| Probability         	|     Prediction	        					| 
|:---------------------:|:---------------------------------------------:| 
| 1.0        			|30Km/h   									| 
|9.938535e-14  |Speed limit (70km/h)|
| 2.28e-14				| Speed Limit(20Km/h)|
| 7.39e-15      			|Speed Limit(50Km/h)|
| 2.3e-17				    |Traffic Signal 							|


For the rest image the model is itself present inside the Last section of main
[Notebook](https://github.com/shashankkmciv18/Self-Driving-Car/blob/master/P3%20Traffic%20Sign%20Classifier/codes/main.ipynb) have a look at that 
### (Optional) Visualizing the Neural Network (See Step 4 of the Ipython notebook for more details)
#### 1. Discuss the visual output of your trained network's feature maps. What characteristics did the neural network use to make classifications?
The output of trained Nueral Network is that it shows an number from 0-42 which basically means that 0 means some thing 1 means something and so on.Neueral Network may have taken various characterstic for this purpose ( basic purposes of the CNN like pattern matching and MaxPooling etc)
![alt text][image9] 

### Approval from Udacity
[Approved]: ./Files/Approved.jpg "Approval"

![alt text][Approved]
