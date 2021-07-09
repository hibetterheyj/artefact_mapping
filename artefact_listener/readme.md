# Listener module for artifact detection & mapping

## Overview

Listen to `Artefact.msg` published from `/W_artefact` topic and export as csv for final evaluation!

## TODOs

- [x] conversion from topics to csv file

  - class: https://github.com/ethz-asl/darknet_catkin/blob/master/data/coco.names

- [x] transformation from odom to map

  use tf2 for transmation

  https://answers.ros.org/question/273205/transfer-a-pointxyz-between-frames/

- [x] tune the parameter for YOLO detection
  <https://github.com/ethz-asl/artefact_mapping>

  track_reassociation_iou	0.3 (team 6 set to 0.15)

  take more time when using YOLO2

- [x] debug visualize with `rqt_image_view`

  publish_debug_images	false -> **to publish a debug topic `/artefact_mapping/debug_image`**

## Code Reading

- debug

  ```sh
  rostopic echo /W_artefact
  ```

- `object-tracking-pipeline.cc`

  ```c++
  // publish_debug_images set as True for puslishing debuging images
  landmark_publisher_ = nh_.advertise<geometry_msgs::PointStamped>("/W_landmark", 1);
  artefact_publisher_ = nh_.advertise<artefact_msgs::Artefact>("/W_artefact", 1);
  ```

- use rqt_tf_tree to visualize

  ```sh
  rosrun rqt_tf_tree rqt_tf_tree
  ```

- Using tf_echo

  ```sh
  # rosrun tf tf_echo [reference_frame] [target_frame]
  rosrun tf tf_echo map odom
  ```

## Dependance installation

- Test passed with following environment

```txt
# pip3 freeze > requirements.txt
# python3 -c "import sklearn; sklearn.show_versions()"
System:
    python: 3.8.10 (default, Jun  2 2021, 10:49:15)  [GCC 9.4.0]
executable: /usr/bin/python3
   machine: Linux-5.8.0-59-generic-x86_64-with-glibc2.29

Python dependencies:
          pip: 21.1.3
   setuptools: 45.2.0
      sklearn: 0.24.2
        numpy: 1.17.4
        scipy: 1.7.0
       Cython: 0.29.14
       pandas: 1.2.5
   matplotlib: 3.1.2
       joblib: 1.0.1
threadpoolctl: 2.1.0

Built with OpenMP: True
```

- install sklearn for clustering close instances in artefact detection [official doc](https://scikit-learn.org/stable/install.html)

  ```sh
  pip3 install -U scikit-learn
  ```

## Misc.

### Required classes in ETHz Robotics Summer School Challenge!

```markdown
0 person
11 stop sign
24 backpack
25 umbrella
39 bottle
74 clock
```