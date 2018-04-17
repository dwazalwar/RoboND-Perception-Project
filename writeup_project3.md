
# 3D Perception Project #

In this project, the main task is to solve perception problem of locating a target object from a collection of objects in a cluttered environment. Also, a 3D map of the environment is to be created from an RGB-D Camera data so that Robot is able to perceive the environment around it. In terms of implementation, this project was divided into 3 different exercises and then combined later to create perception pipeline.

## RGB-Camera ##

RGB-Data camera is a hybrid sensor, where apart from a passive RGB camera, we also have an active depth sensor in form of infrared (IR) projector and receiver. IR transmitter projects light out in a predefined pattern like simple stripes or even at times we have unique convoluted patterns and the receiver analyzes the reflected light from the object surface. Depth is perceived by interpreting the deformation in the pattern caused by the surface of target objects. The main advantage in using RGB-D camera for 3D perception is that now we have per pixel information available at a relatively low cost and also computationally inexpensive in comparison to other sensors like stereo cameras. Its simple USB interface allows its use in various applications including complex mapping to any other object recognition tasks.

## Point Cloud ##

Point Cloud is a digital representation of 3D Objects, where we have additional metadata for each point and also some useful methods for operating on the point cloud. Metadata can be in form of RGB values for each point, intensity values or even local curvature information. Methods are in form of some useful functionality like iterators for traversing points, filtering functions or any other statistical analyzes on the cloud. Typical Point cloud types available in powerful Point Cloud Library (PCL) are PointXYZ, PointXYZI, PointXYZRGB, PointNormal and many such other forms. Selection of point cloud type is based on factors like project goals, sensor types and also on actual objects included in perception tasks. For this project, we are using PointXYZRGB.

## Exercise 1: Filtering and RANSAC plane fitting ##

Even after calibration, external factors like dust, humidity in air or presence of various light sources can create sparse outliers in point cloud data. The presence of such outliers can cause perception pipeline to fail in a very noisy scenario as it becomes difficult to estimate point cloud characteristics like curvature, gradient, causing erroneous values. PCL's StatisticalOutlierRemoval filter is a powerful tool to solve this issue. Depending on the threshold scale factor set, if the mean distance of its neighbors is more than interval defined by the global distances mean+standard deviation, that point in point cloud is considered an outlier. For this project, after some manual trials, a value of 0.1 was set for threshold scale factor. Image below shows original noisy point cloud data and result after outlier removal filter.

[image_1]: comparison.png
![alt text][image_1]

Working on the entire 3D point cloud data obtained after outlier filtering is computationally expensive and also inefficient. Also, point cloud data created from RGB-D cameras already is very dense and feature-rich, so even with some downsampling, enough information is available for further perception steps. This is where VoxelGrid Downsampling Filter comes into the picture, which is used to downsample original point cloud data by taking a spatial average of the points in the cloud confined by each voxel. The key performance metrics here is the voxel size also referred as leaf size and its unit meters. After experimenting a bit, value 0f 0.005 seemed good enough to retain important details in point cloud after downsampling.

Passthrough filter is a great tool to have in cases where we have some prior information about target location in our scene. It is like a cropping tool where we define our region of interest called passthrough by specifying an axis with cut-off values along that axis. For this project, I have used 2 successive pass-through filters. One along z-axis with limits 0.6 < z < 1.1 to retain only the tabletop and the objects sitting on the table. The second one is along y-axis with limits -0.5 < y < 0.5 to remove the dropbox edges from our scene. Image below shows the output after applying these 2 passthrough filters.

[image_2]: passthrough.png
![alt text][image_2]

After filtering, we still need to remove the table from the scene and for that, we can use Random Sample Consensus or "RANSAC". RANSAC assumes point cloud data is comprised of inliers and outliers. Depending on the model defined, inliers are the points in the point cloud which fit the model and thereby outliers can be defined as the points which do not fit the model. The model we define can be in any form like plane, cylindrical or any common shape. In this case, we know the table is rectangular and so here we use PCL's RANSAC plane fitting. Since table here is the single most prominent plane in the scene, even with few iterations of RANSAC algorithm we can segment table out from the scene. However in other cases where we have some other planes in the scene, for an optimal solution we may have to run additional iterations resulting in more compute time. Image below shows the output after RANSAC plane fitting.

[image_3]: objects.png
![alt text][image_3]

## Exercise 2: Clustering for segmentation ##

So far we have performed RANSAC plane fitting using shape information of the objects in the scene. We may continue to perform RANSAC fitting to further segments individual objects using cylindrical, spherical or rectangular models. However, this will need to do multiple runs on whole point cloud data which itself is inefficient and also just relying on one feature information usually does not give us a robust solution. Our 3D point cloud data has some other details like color, spatial information or any such rich features, which can be used for our segmentation task. One such unsupervised learning technique called Clustering is widely used in perception, where the main idea is to find similarities among individual points and cluster them into different groups.

In this project, we have used Density-Based Spatial Clustering of Applications with Noise, also called "DBSCAN Algorithm" or sometimes even as “Euclidean Clustering”. DBSCAN creates clusters by grouping data points that are within some threshold distance from the nearest another point in the data. Even though here we use spatial neighborhood information as its feature for calculating Euclidean distance, it's not the only available feature. We can use any other feature like say color and define Euclidean distance calculation in terms of that feature, it should still work. One important advantage of DBSCAN in comparison to its counterpart algorithm kmeans is that DBSCAN works well even in cases where expected number fo clusters are unknown. Also, kmeans performance is dependant on its convergence criteria and also the number of iterations, DBSCAN does not suffer from this issue.

In terms of using PCL's Euclidean Clustering algorithm for this project, there are 3 important parameters affecting overall performance. First is ClusterTolerance, which is the allowed spatial cluster tolerance as a measure in the L2 Euclidean space. After several trials, I decided to keep a value of 0.05 as cluster tolerance. Second and Third parameters are Min and Max cluster sizes, which are set based on the minimum and maximum size of target objects in the scene we are trying to segment. For this project, using minClusterSize = 50 and maxClusterSize = 5000, I was able to segment all objects in the scene. Since PCL's Euclidean CLustering only supports k-d trees, we have to convert point cloud XYZRGB data first in XYZ form and from it, we can construct k-d tree data structure. For visualization in Rviz, we create a separate cloud with unique color assigned to each cluster and this cloud after converting in ROS form is then published. Image below shows the output after clustering step.

[image_4]: clustering.png
![alt text][image_4]

## Exercise 3: Object Recognition ##

To segment the specific object from clustered objects, we need an object recognition algorithm. In object recognition step, we characterize features that can be used to uniquely describe our target object and can then be used to separate them from other objects in the scene. One such feature available in our point cloud data is color information RGB, however, this information is sensitive to lighting conditions since objects can appear to have different colors under varying lights. To avoid this we can convert RGB information in HSV color space using openCV and make our feature robust. Color information is converted into normalized histogram form to accommodate varying image sizes for a target object. Another feature which can be used is the partial information of 3D shapes available in the object point cloud data. Object surface can be described by its distribution of surface normals and this in normalized form can be used a feature. Details for feature extraction in this project can be seen features.py

For classification purpose, we use a supervised learning technique called Support Vector Machine "SVM". In SVM, we plot each data item as a point in n-dimensional space (where n is the number of features) and then look for hyperplane to separate them into target classes. To use SVM in scikit-learn, we have to create a labeled dataset for the target as well non-target objects. For this project's implementation default linear kernel in SVM worked very well. For best visualisation of training and validation results, a normalized confusion matrix was obtained. Confusion Matrix is a powerful metrics used in a classification problem, and it has details intra-class details like false positives, true positives, false negatives and true positives.

Object list for classification was chosen from pick_list_3.yaml as it had all the objects in 3 test scenarios. To improve accuracy, I tried a different set of a randomly generated orientation of objects. The default implementation in capture_features.py had 5 but I tried 10, 25, 50, 100 and 200. After evaluating results, 200 seemed good enough number. This had maximum impact in terms of improving the training model and with just increasing data I was able to reach an accuracy of 95%. I also tried both RGB and HSV implementation and found out that HSV had a slight edge in terms of performance. In terms of histogram bin size, I found 64 is the best value in terms of real-world performance. Normalized confusion matrix after SVM training can be seen in image below

[image_5]: normalized_confusion_matrix.png
![alt text][image_5]

## Pick and Place Setup ##
All 3 above exercises were combined together to form perception pipeline pcl_callback() for this project. The object recognition task gave 100% accuracy in all 3 test scenarios. Output images from Rviz can be seen below for all 3 test cases.

[image_6]: test_scene_1.png
![alt text][image_6]

[image_7]: test_scene_2.png
![alt text][image_7]

[image_8]: test_scene_3.png
![alt text][image_8]

For creating output yaml files required for pick_place operation we need certain information like test scene number, object name, arm name, pick pose and place pose. For each detected object in scene, if it matches with an expected pick list, we create these messages and finally convert it into a dictionary using make_yaml_dict()

## Further Improvements ##

In terms of further improvements, I would like to different explore SVM kernels like rbf, polynomial and see how it impacts object recognition performance, especially in complex scenes. Also, I feel thresholds for statistical outlier filtering may need more optimization in case of noisy and cluttered environments.

I would also like to work on additional challenges, work on collision mapping and using the pick_place_server to execute the pick and place operation






