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

#include "topic_to_image/topic_to_image.hpp"

TopicToImage::TopicToImage(const rclcpp::NodeOptions & options) :
  Node("topic_to_image", options)
{
  // IO
  input_topic_ = this->declare_parameter<std::string>("input/topic", "/image_raw");
  output_path_ = this->declare_parameter<std::string>("output/path", "/tmp/");
  file_prefix_ = this->declare_parameter<std::string>("output/prefix", "");

  bool compressed = this->declare_parameter<bool>("compressed", false);
  std::string image_transport = "raw";
  if (compressed) {
    image_transport = "compressed";
  }
  image_sub_ = image_transport::create_subscription(this,
                                                    input_topic_,
                                                    std::bind(&TopicToImage::ImageCallback, this, std::placeholders::_1),
                                                    image_transport,
                                                    rmw_qos_profile_sensor_data
  );
  RCLCPP_INFO_STREAM(get_logger(), "Using image_transport: '" << image_transport << "' on " << image_sub_.getTopic());
  if(output_path_.empty()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Invalid Path provided:" << output_path_ << ". Terminating...");
    rclcpp::shutdown(nullptr, "Invalid Output Path");
  }
  if (!std::filesystem::exists(output_path_)) {
    RCLCPP_INFO_STREAM(get_logger(), "The Provided path [" << output_path_ << "]doesn't exist. Trying to create.");
    if (std::filesystem::create_directories(output_path_)){
      RCLCPP_INFO_STREAM(get_logger(), "The Provided Path was created successfully.");
    }
    else {
      RCLCPP_ERROR_STREAM(get_logger(), "Could not create the output directory: " << output_path_ << ". Terminating");
      rclcpp::shutdown(nullptr, "Missing Permissions on the output path");
    }
  }
  RCLCPP_INFO_STREAM(get_logger(), "Saving Images PNGs to:" << output_path_ << ",  with prefix:" << file_prefix_);
}

void TopicToImage::ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg)
{
  std::string fname;
  fname = output_path_ + "/" + file_prefix_ + "_" +
      boost::lexical_cast<std::string>(image_msg->header.stamp.sec) + "."+
          boost::lexical_cast<std::string>(image_msg->header.stamp.nanosec) + ".png";

  cv_bridge::CvImagePtr in_image_ptr;
  try {
    in_image_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  cv::imwrite(fname, in_image_ptr->image);

  RCLCPP_INFO_STREAM(get_logger(), "Image saved to: " << fname);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(TopicToImage)


