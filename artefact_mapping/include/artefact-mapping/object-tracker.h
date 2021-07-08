#ifndef OBJECT_TRACKER_OBJECT_TRACKER_H_
#define OBJECT_TRACKER_OBJECT_TRACKER_H_

#include "artefact-mapping/object-view.h"
#include "artefact-mapping/tracker-utils.h"

#include <Eigen/Core>
#include <iostream>
#include <map>
#include <memory>
#include <queue>
#include <vector>

#include <darknet.h>
#include <image_opencv.h>
#include <parser.h>
#include <ros/ros.h>

#include <kcftracker.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class Observation {
public:
  Observation(ros::Time timestamp, unsigned x, unsigned y, int class_label)
      : timestamp_(timestamp), x_(x), y_(y), class_label_(class_label) {};
  const Eigen::Vector2d getCentroid() const {
    Eigen::Vector2d centroid;
    centroid << x_, y_;
    return centroid;
  };
  const int getClass() const {
    return class_label_;
  };
  ros::Time timestamp_;
  unsigned x_, y_;
  int class_label_;
};

class ObjectTracker {
public:
  explicit ObjectTracker(unsigned detector_period);
  ~ObjectTracker();

  void processFrame(const cv::Mat &frame_bgr, const ros::Time &timestamp, const float& yaw);
  void debugDrawTracks(cv::Mat *frame_bgr);
  bool getFinishedTrack(std::vector<Observation> *observations);

private:
  unsigned frame_count_;

  unsigned detector_period_;
  darknet::network net;

  unsigned last_track_id_;
  std::unordered_map<unsigned, std::unique_ptr<KCFTracker>> trackers_;
  std::unordered_map<unsigned, ObjectView> track_heads_;
  std::unordered_map<unsigned, std::vector<Observation>> tracks_;

  std::map<int, float> track_yaws_;

  std::queue<unsigned> finished_tracks_;

  std::vector<int> detected_classes_;
};

#endif // OBJECT_TRACKER_OBJECT_TRACKER_H_
