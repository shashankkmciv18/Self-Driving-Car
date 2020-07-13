# **Finding Lane Lines on the Road** 

## Writeup Template

### You can use this file as a template for your writeup if you want to submit it as a markdown file. But feel free to use some other method and submit a pdf if you prefer.

---

**Finding Lane Lines on the Road**

The goals / steps of this project are the following:
* Make a pipeline that finds lane lines on the road
* Reflect on your work in a written report


[//]: # (Image References)

[image1]: ./examples/grayscale.jpg "Grayscale"

---

### Reflection

### 1. Describe your pipeline. As part of the description, explain how you modified the draw_lines() function.

My pipeline consisted of 7 steps.  
1.Grayscaled the image </br>
2.Apply Canny Egde</br>
3.Apply Gaussian Blur</br>
4.Use Region of intrest concept</br>
5.Apply hough lines </br>
6.Find best fit lines </br>
7.Draw lines </br>
Have a look in code itself please there i have explained everythnig with detail
In order to draw a single line on the left and right lanes, I modified the draw_lines() function by ...

If you'd like to include images to show how the pipeline works, here is how to include an image: 

![alt text][image1]


### 2. Identify potential shortcomings with your current pipeline


One potential shortcoming would be what would happen when ... 

Another shortcoming could be ...

Here's a [link to my SolidWhiteRight video result](./test_videos_output/solidWhiteRight.mp4)</br>
Here's a [link to my Soldigyellowleft video result](./test_videos_output/solidYellowLeft.mp4)</br>

### 3. Suggest possible improvements to your pipeline

Using Bird eye view so that an better picture will come
