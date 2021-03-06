#pragma once
// SYSTEM
#include <chrono>
#include <iostream>
// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "std_msgs/msg/string.hpp"
// OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaarithm.hpp>
#include "opencv2/core.hpp"
#include "opencv2/core/cuda.hpp"
#include "opencv2/cudaimgproc.hpp"

// PROJECT
#include "camera_interfaces/msg/depth_frameset.hpp"

#include "Timer.h"

/**
 * @brief Image viewer node class for receiving and visualizing fused image.
 */
class BackgroundImageNode : public rclcpp::Node
{
	typedef std::chrono::high_resolution_clock::time_point time_point;
	typedef std::chrono::high_resolution_clock hires_clock;

public:
	BackgroundImageNode();
	void init();

private:

	cv::cuda::GpuMat m_depth_image_cuda; 
	cv::cuda::GpuMat m_color_image_cuda; 
	cv::cuda::GpuMat m_color_image_cuda_tmp; 

	std::string m_window_name_frameset_0		= "color";
	std::string m_FPS_STR = "";
	int const m_max_binary_value = 100000;
	int m_threshold_type = 4;
	bool m_print_fps;
	uint64_t m_frameCnt = 0;

	Timer m_timer;        // Timer used to measure the time required for one iteration
	double m_elapsedTime; // Sum of the elapsed time, used to check if one second has passed

	int m_image_rotation, m_depth_thr;

	uint8_t *m_send_frame_bytes = NULL;

	rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
	OnSetParametersCallbackHandle::SharedPtr callback_handle_;

	cv::Ptr<cv::cuda::Filter> m_gaussian;

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_image_publisher	= nullptr;

	rclcpp::QoS m_qos_profile = rclcpp::SystemDefaultsQoS();

	rclcpp::Subscription<camera_interfaces::msg::DepthFrameset>::SharedPtr m_frameset_subscription;

	void framesetCallback(camera_interfaces::msg::DepthFrameset::UniquePtr fset_msg);
	void publishImage(uint8_t * color_image, int width, int height, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr message_publisher);

	cv::Mat rotateAndScale(cv::cuda::GpuMat& input);
	void blurAndRemoveBackground(cv::cuda::GpuMat& color_input, cv::cuda::GpuMat& depth_input);

	void CheckFPS(uint64_t* pFrameCnt);
	void PrintFPS(const float fps, const float itrTime);

};
