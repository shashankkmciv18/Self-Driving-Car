
**Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image1]: ./c5/c410.png "Undistorted"
[image2]: ./c2/t247.jpg "Road Transformed"
[image3]: ./c3/t340.jpg "Binary Example"
[image4]: ./c2/t240.jpg "Warp Example"
[image5]: ./c2/Lanepixel.jpg "Fit Visual"
[image6]: ./c4/t311.png "Output"
[video1]: ./project_video.mp4 "Video"

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Writeup / README

### Camera Calibration

#### 1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.
The code acan be located in output.inpyb in an fucntion called callibrate 

I start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world. Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image.  `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection.  

I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image using the `cv2.undistort()` function and obtained this result: 

![alt text][image1]

### Pipeline (single images)

#### 1. Provide an example of a distortion-corrected image.
First we have applied distortion correction

#### 2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.

I used a combination of color and gradient thresholds to generate a binary image (thresholding steps at lines # through # in `another_file.py`).  Here's an example of my output for this step.  (note: this is not actually from one of the test images)
In this  i have only used the HSV and RGB thresholding other technquies like sobel were not used as they were not giving an good result

![alt text][image3]

#### 3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

The code for my perspective transform includes a function called `warp()`, which  apperas in output.inpyb There are 3 different warp function 2 of which were not giving satisfying results.
Third warp(which is just present as code gives an satisfactory result).  The `warp()` function takes as inputs an image (`img`).  I chose the hardcode the source and destination points in the following manner:

```python
 src = np.float32(
    [[((img_size[0] / 6) - 10), img_size[1]],[(img_size[0] / 2) - 55, img_size[1] / 2 + 100],
    [(img_size[0] / 2 + 55), img_size[1]/ 2 + 100],[(img_size[0] * 5 / 6) + 60, img_size[1]]]
    )

    dst=np.float32([[240,720],[240,0],[1000,0],[1000,720]])
```

This resulted in the following source and destination points:

| Source        | Destination   | 
|:-------------:|:-------------:| 
| 170,720      | 240,720        | 
| 485, 460      | 240, 0      |
| 595, 460     | 1000, 0      |
| 960, 720      | 1000, 720        |

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.
This function returns an warp immage and 2 Matrix calles as M and Minv,whish are just transformation matrices from cource to sedtimnation and destination to source respectively.

![alt text][image4]

#### 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?
I have plotted an histogram on the image and then find the pints and then using this i have found out the Polynomial and the positions

![alt text][image5]

#### 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

I did this in lines # through # in my code in `my_other_file.py`

#### 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

I implemented this in the same file with the finction named as ` in the function `draw_lines()`.  Here is an example of my result on a test image:

![alt text][image6]

---

### Pipeline 
1.calibrated the caamera </br>
2.Undistort the image after points obtained from calibration However i was not using undistorted image in final pipeline as this made my code very slow </br>
3.mask the image using The function color_transform(out put is and maskedimage) </br>
4.Warp the image(i.e Undistorted image outputs are warped image,adn 2 matrices) </br>
5.Create an warptransformation to the warped image with Minv matrics for giving it the same porcepective </br>
6.apply color transformation to the warpedimage( using the function coolor_transformation) </br>
7.get an output image after using addWeight funcntion to the undistorted image and new warp image </br>
8.Fit polynomial usinf function fir_polynomial whise inputs is the image obtained in 6th step </br>
9.Calculate the curavature usinf function known as measure_curvature_real </br>
10.Draw lines on undistorted image these are the lines for lanes,This is output </br>
11.put   texts like curvature raddius and centre deflection to the given image </br>
12.I also have appended some of the image for an good visualisation </br>
13.Get output image and other required images , in my case they are an image containng image of output   4 and 7, another image of containg output of step 3 and 6, left curvature and right curvature         values outpur of step  6 and the Ouputt of step 10 i.e the main ouput </br>
14.Display it using opencv
# Please open [Code_for_Video.ipynb](https://github.com/shashankkmciv18/Self-Driving-Car/blob/master/P2%20Advance%20Lane%20%20Detection/Code_for_Video.ipynb) file for the updated and more clear verison of code.</br>
[Complete_code.ipynb](https://github.com/shashankkmciv18/Self-Driving-Car/blob/master/P2%20Advance%20Lane%20%20Detection/Complete_code.ipynb) consist of more raw code and a rough code that works, please view [this file](https://github.com/shashankkmciv18/Self-Driving-Car/blob/master/P2%20Advance%20Lane%20%20Detection/Code_for_Video.ipynb) for clear version of code.
#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

Here's a [link to my video result](./output_video/Finaloutput.avi)

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

One of the major challenge faced in this was finding an add coordinate for warping as that involves a lot of practise/another was when i was tryng to unwarp the image of other dimenson to this dimesnion however that method was not applied here.
This pipeline will mainly fail when there is a very sharp turn which will result in non detection of the lane and hence it will fail.
I am trying to implement sliding window algorithm for making  it even more robust another technique which i am thinking is to use Deep learning model as using open cv is relatively slower to that.
I have an rough sketch regarding this using like using Mixed Gaussian Ditribution alongside of Kalmanfilter and reverse Pid

### Approval from Udacity
[Approved]: ./Files/Approved.jpg "Approval"

![alt text][Approved]
