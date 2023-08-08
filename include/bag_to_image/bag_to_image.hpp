/*
 *  Copyright (c) 2023, MAP IV.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  v1.0 amc-nu 2023-08
 */

#ifndef TOPIC_TO_IMAGE_H
#define TOPIC_TO_IMAGE_H

#include <memory>
#include <algorithm>
#include <regex>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "boost/lexical_cast.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/time.h"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "sensor_msgs/msg/compressed_image.h"
#include "sensor_msgs/msg/image.hpp"


class BagToImage : public rclcpp::Node
{
public:
  explicit BagToImage(const rclcpp::NodeOptions & options);

private:

  image_transport::Subscriber image_sub_;

  std::string output_path_, input_path_, bag_format_, storage_id_;
  std::vector<std::string> input_topics_;
  void CheckParams();

  void ReadBag();

  void CreateDirectories();

  cv_bridge::CvImagePtr
  MessageToImage(std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag_message, const std::string &topic_type);
};

#endif //TOPIC_TO_IMAGE_H
