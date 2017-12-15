
# Writeup

## Project: Perception Pick & Place

--------------------------------------------------------------------------------

#### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

--------------------------------------------------------------------------------
### Some comments about the code

The code for the **RoboND-Perception-Excercise** is at the `row_ws/3-catking_ws/` specifically the for `sensor_stick`

The code for the current project is mainly in `pr2_perception.py`. The other important file is `my_helper.py` that has all the python functions implemented and used in` pr2_perception.py`.

The `pick_place_project.launch` file receive as argument `num` the test scena number.

  * i.e. `roslaunch pr2_robot pick_place_project.launch num:=1` for test1.world and `pick_list_1.yaml`.

The same is for the python script `pr2_perception.py` that recieve the argument -n  o --number.

 * i.e `python pr2_perception.py -n 1` for the test1.world

### Exercise 1, 2 and 3 pipeline implemented

##### 0\. Add statistical filter clean up the noisy point cloud data.

In order to achieve the cleaning of noisy data, a function was created that filters out statistical outliers.

![**Noisy point cloud.**](./img/raw.png)

From the premise:

>Assuming a Gaussian distribution, all points whose mean distances are outside an interval defined by global mean+standard deviation distances are considered outliers and removed from the point cloud.

Results by `k_neighbors=10, scale_f=0.05`:


![**Cleaned up point cloud after filter.**](./img/clean.png)


#### 1\. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

Exercise 1 consisted of applying RANSAC to process the data from the point cloud and extract only the points that are part of the objects by adjusting the data to a flat model.  

The steps to complete this exercise are the following:

  1. Downsample your point cloud by applying a Voxel Grid Filter.
    - Leaf size of the voxel: **0.01**
  2. Apply a Pass Through Filter to isolate the table and objects.
    - Axis **z**, between: **0.57** and **1.1**
    - Axis **y**, between: **-046** and **0.46**
    - Axis **x**, between: **0.4** and **1**
  3. Perform RANSAC plane fitting to identify the table.
  4. Use the ExtractIndices Filter to create new point clouds containing the table and objects separately.

![**RANSAC plane fitting to identify the table.**](./img/table.png)


![**Tabletop objects separated after applying RANSAC.**](./img/voxel_grid.png)

#### 2\. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.

Work was done in Exercise 2 to separate the point clouds corresponding to each object and then assign them a color, obtaining the following results:

The steps to complete this exercise are the following:

  1. Apply Euclidean clustering on the table-top objects (after table segmentation is successful)
      * As the different size of the objects, there were also different size of clusters:
          - It was choose a tolerance of **0.05**.
          - Cluster size between **50** and **8000**
  2. Create a XYZRGB point cloud such that each cluster obtained from the previous step has its own unique color.
  3. Finally publish your colored cluster cloud on a separate topic clusters

![**Colored clusters**](./img/clustering&color.png)

#### 3\. Complete Exercise 3 Steps. Features extracted and SVM trained. Object recognition implemented.

Exercise 3 asked to classify the cluster for object recognition. To do that it was necessary to:

  1. Generate a training set by capturing features. `training_set_2.sav`
    * 500 samples per object
    * 32 bins in a range of 0 to 256 to calculate the color histograms.
    * 10 bins in a range of -1 to 1 to calculate the normal histograms.
  2. Train a support vector machine for classification.
    * After testing different configurations of the SVM it was opted for the linear kernel and C=1.2.

![**Confusion Matrix**](./img/cmx.png)

>There also were created:
>
> `training_set.sav`
>    * 50 samples per object
>    * 32 bins in a range of 0 to 256 to calculate the color histograms.
>    * 10 bins in a range of -1 to 1 to calculate the normal histograms.
>
> `training_set_3.sav`
>    * 500 samples per object
>    * 64 bins in a range of 0 to 256 to calculate the color histograms.
>    * 10 bins in a range of -1 to 1 to calculate the normal histograms.
>But not good enough results.

  3. Classify the clouds of segmented objects and publish label markers.

![**3/3 Test_world_1**](./img/clustering&color.png)

![**4/5 Test_world_2**](./img/test2.png)

![**7/8 Test_world_3**](./img/test3.png)

#### 4\. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object. Then create a ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).

To calculate the centroid, we worked with the point clouds and the data type. Then for each cluster the mean of each axis was calculated to obtain the coordinates of the centroid.

Finally it was created the ROS messages required to write a `output_1-3.yaml` for each test scene. For each individual object it was written the follow information:

```yaml
object_list:
- test_scene_num: 1
  arm_name: left
  object_name: car
  pick_pose:
    position:
      x: 0
      y: 0
      z: 0
    orientation:
      x: 0
      y: 0
      z: 0
      w: 0
  place_pose:
    position:
      x: 0
      y: 0
      z: 0
    orientation:
      x: 0
      y: 0
      z: 0
      w: 0
```

## Conclusion

All the concepts and techniques acquired during exercises 1, 2 and 3 were applied successfully. A statistical filter was also applied to clean the intrusive data with acceptable results. Finally, all the necessary ROS messages were created to obtain the appropriate information to write the yaml files as requested.

The classification was relatively good, but he had problems with the book classifying it as sticky notes. That could be solved with a larger training set, working with the capture of features, p. change the histogram settings or adjust the SVM parameters.

As a curious note, when the voxel size is 0.005, the SVM usually confuses the glue with the cookies but correctly recognizes the book. In addition, the output was not stable, so a small logic was added to wait until 3 equal consecutive exits remain. As an example, with the previous configuration of the voxel grid filter, the SVM manages to correctly classify all the objects as shown in the following image and the `output_3_100.yaml`.

![**8/8 classification with LEAF_SIZE=0.0005**](./img/test_3_100.png)

As future work, it is planned to face the challenge of pick and place.
