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

Note that the top confusion matrix is normalized so that the sum of each row is 1, while the bottom matrix is not normalized. The diagonal can be interpreted as the probability of object being correctly classified, and the off-diagonal are the probability of false-positive and false-negative. 


## Combining techniques in Exercises and use in ROS PR2 robot
In the `pr2_pap` ROS node, we subscribe point cloud data as we have done in Exercise 2 and 3 and process the data in callback function `pcl_callback`. We can reuse the same code as in Exercise 2 and 3 (with a little tweak for `set_clusterTolerance`, `set_MinClusterSize`, and `set_MaxClusterSize`). A list of detected object labels and associated point clouds data are generated and passed to `pr2_mover()`, where we parse a list of objects that are required to recognize and pick up. Figures below shows successful object recognition in test scene 1, 2, and 3, respectively.

![test1]
![test2]
![test3]

The recognition accuracy for each test case is 100%, 100%, and 87.5%, respectively. In test scene 3, glue is occluded by the book, which poses difficulty in clustering, resulting in recognition failure. 

Finally, we convert required parameters (`test_scene_num`, `arm_name`, `object_name`, `pick_pose`(centroid of the object), `place_pose` (dropbox location)) into ROS format and pass to ROS service `pick_place_routine`, which tells PR2 to execute the actions. 

To generate the `yaml` file, `make_yaml_dict()` is called prior to the ROS service call to create a list of dictionary for each target object. At the end, the dictionary is passed to `send_to_yaml()` to produce the `yaml` file.

### Future improvement
1. Improve object recognition using different SVM kernel functions
2. Implement robust pick and place kinematics for both arms
3. Use neural network for object recognition techniques 