## Project: Perception Pick & Place
### This project demonstrates the pipeline of robotic perception: point cloud downsampling, filtering, clustering, and classification. The pipeline is tested under total of three scenarios: 3 objects, 5 objects, and 8 objects in the scene, respectively. 

[//]: # (Image Reference)

[test1]: ./img/test1.png
[test2]: ./img/test2.png
[test3]: ./img/test3.png
[pcl_leaf_size_001]: ./img/pcl_leaf_size_001.png
[pcl_leaf_size_01]: ./img/pcl_leaf_size_01.png
[no_passthrough_filter]: ./img/no_passthrough_filter.png
[with_passthrough_filter]: ./img/with_passthrough_filter.png
[extracted_inliers]: ./img/extracted_inliers.png
[extracted_outliers]: ./img/extracted_outliers.png
[clustering_no_edge]: ./img/clustering_no_edge.png
[feature_capture_1]: ./img/feature_capture_1.png
[feature_capture_2]: ./img/feature_capture_2.png
[feature_capture_3]: ./img/feature_capture_3.png
[confusion_matrix_w_norm]: ./img/confusion_matrix_w_norm.png
[confusion_matrix_wo_norm]: ./img/confusion_matrix_wo_norm.png

---
## Project Procedures: Exercise 1

### Point Cloud Voxel Downsampling
Raw point cloud has too many points for any postprocessing. Hence, we use voxel downsampling to reduce the amount of data points. Here, leaf (aka, voxel size) of 0.01 is used to average the points within each voxel:

![pcl_leaf_size_001]

### Passthrough Filtering
Because we only focus on the objects above the table, we apply passthrough filter to eliminate the points besides the table plank and the objects above it. Through experiment, the optimal range of visibility is between z=0.6 and z=1.1:

![with_passthrough_filter]

### Outlier rejection via RANSAC
RANSAC algorithm is leveraged to fit a model to the data. The data close to the model within certain tolerance are classified as inliers, otherwise as outliers. In our case, we fit a plane to the point cloud after passthrough filtering to obtain the data point for the table. By toggling the parameter in the `cloud_filtered.extract()` function, we can also obtain the data points that represent the objects above the table.

![Outliers: objects][extracted_outliers]
![Inliers: table][extracted_inliers]

## Project Procedures: Exercise 2
### K-mean Clustering
Since applying RANSAC to every single item is extremely inefficient to identify individual object, instead we use k-mean clustering algorithm to group the points into categories. K-mean is an unsupervised machine learning algorithm by randomly assigning class mean and iterating the expectation calculation process till result convergence. A color is assigned to each class for visualization.

![clustering_no_edge]

```
ec = objects_xyz.make_EuclideanClusterExtraction()
ec.set_ClusterTolerance(0.02)
ec.set_MinClusterSize(115)
ec.set_MaxClusterSize(1300)
```
We use Eclidean distance metric and some predefined parameters to find optimal class mean. 

## Project Procedures: Exercise 3
### Object Classification
Once points are correctly grouped into appropriate object boundaries, we can classified the object base on the its color and normal feature. In this project, each item has its unique color histogram, espectially in HSV, or Hue-Saturation-Value, color space, and surface normal distribution. We can first build the color and normal histogram:

```
chists = compute_color_histograms(ros_cluster, using_hsv=True)
normals = get_normals(ros_cluster)
nhists = compute_normal_histograms(normals)
feature = np.concatenate((chists, nhists))
```
Both histograms are concatenated into feature vectors, composing a feature space, where SVM (Support Vector Machine) is used to predict object labels. Prior to prediction, SVM needs to be trained in order to build a model of how the objects look like. By running `capture_features.py`, each object is examined at multiple angle view. 

![feature_capture_1]
![feature_capture_2]

When feature capture is finished, a model file is generated, which is to be later used for confusion matrix and pr2_robot classification task. 

Once we have `model.sav` from `capture_features.py`, we can run `train_svm.py` to generate SVM model accuracy and [confusion matrix](https://en.wikipedia.org/wiki/Confusion_matrix) as figures below:

![confusion_matrix_w_norm]
![confusion_matrix_wo_norm]









1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  

#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.
Here is an example of how to include an image in your writeup.

![demo-1](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

And here's another image! 
![demo-2](https://user-images.githubusercontent.com/20687560/28748286-9f65680e-7468-11e7-83dc-f1a32380b89c.png)

Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.  



