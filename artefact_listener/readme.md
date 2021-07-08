# Listener module for artifact detection & mapping

## TODOs

- [x] conversion from topics to csv file

  - class: https://github.com/ethz-asl/darknet_catkin/blob/master/data/coco.names

- [x] transformation from odom to map

  use tf2 for transmation

  https://answers.ros.org/question/273205/transfer-a-pointxyz-between-frames/

- [ ] add offset to regulate the position!

- [ ] tune the parameter for YOLO detection
  <https://github.com/ethz-asl/artefact_mapping>

  track_reassociation_iou	0.3 (team 6 set to 0.15)

  take more time when using YOLO2
  
- [ ] debug visualize with `rqt_image_view`

  publish_debug_images	false -> **to publish a debug topic `/artefact_mapping/debug_image`**

## Code Reading

- debug

  ```
  rostopic echo /W_artefact
  ```

  

- `object-tracking-pipeline.cc`

  ```c++
  # publish_debug_images set as True for puslishing debuging images
  
  landmark_publisher_ = nh_.advertise<geometry_msgs::PointStamped>("/W_landmark", 1);
  artefact_publisher_ = nh_.advertise<artefact_msgs::Artefact>("/W_artefact", 1);
  ```

- use rqt_tf_tree to visualize

  ```
  rosrun rqt_tf_tree rqt_tf_tree
  ```

- Using tf_echo

  ```
  # rosrun tf tf_echo [reference_frame] [target_frame]
  rosrun tf tf_echo map odom
  ```



