# simple_object_recognition_demo
A simple object recognition demo using PCL filtering and CNN for classification.
This demo uses a live camera, Intel RealSense d435 camera.
It can classify 12 objects, the time to gather the data and train was less then 20 minutes. For each object around 10 image where taken, and trained with augmentations (Different cropping paddings and scaling the Value (brightness) of the images). 
For classification PCL is used to remove the floor (flat surface), cluster the objects, and for each object it tries to find the bounding box in the RGB Image by matching points from the cluster to the original Point Cloud and using the index values to find the position in the RGB Image. 

In the folder './data/training/images/object_name' are the images used for training (excluding augmentation). 


