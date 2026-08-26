#include <gtest/gtest.h>

#include <vision_msgs/msg/detection3_d_array.hpp>

#include "g1_manipulation/manipulation_server.hpp"

TEST(DetectionLookup, UsesFrontResultClassIdAndRejectsInvalidInput)
{
  vision_msgs::msg::Detection3DArray objects;
  vision_msgs::msg::Detection3D detection;
  vision_msgs::msg::ObjectHypothesisWithPose result;
  result.hypothesis.class_id = "cube";
  detection.results.push_back(result);
  objects.detections.push_back(detection);
  EXPECT_TRUE(g1_manipulation::findDetection(objects, "cube"));
  EXPECT_FALSE(g1_manipulation::findDetection(objects, "other"));
  EXPECT_FALSE(g1_manipulation::findDetection(objects, ""));

  vision_msgs::msg::Detection3D empty;
  objects.detections.insert(objects.detections.begin(), empty);
  EXPECT_TRUE(g1_manipulation::findDetection(objects, "cube"));
}
