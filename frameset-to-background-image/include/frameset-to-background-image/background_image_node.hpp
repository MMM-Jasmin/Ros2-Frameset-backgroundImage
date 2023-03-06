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

	float m_maxFPS;

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
	uint8_t *m_send_frame_full_bytes = NULL;
	//uint8_t *m_send_frame_small_416_bytes = NULL;
	//uint8_t *m_send_frame_small_608_bytes = NULL;
	uint8_t *m_send_frame_small_640_bytes = NULL;
	//uint8_t *m_send_frame_small_full_416_bytes = NULL;
	//uint8_t *m_send_frame_small_full_608_bytes = NULL;
	uint8_t *m_send_frame_small_full_640_bytes = NULL;
	uint8_t *m_send_frame_small_full_640_kar_bytes = NULL;
	uint16_t *m_send_depth_frame_bytes = NULL;


	int m_out_small_width = 608;
	int m_out_small_height = 608;

	rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
	OnSetParametersCallbackHandle::SharedPtr callback_handle_;

	cv::Ptr<cv::cuda::Filter> m_gaussian;

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr 	m_image_publisher						= nullptr;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr 	m_image_full_publisher		   			= nullptr;
	//rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr 	m_image_small_416_publisher 			= nullptr;
	//rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr 	m_image_small_608_publisher 			= nullptr;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr   m_image_small_640_publisher 			= nullptr;
	//rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr 	m_image_small_full_416_publisher 		= nullptr;
	//rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr 	m_image_small_full_608_publisher		= nullptr;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr   m_image_small_full_640_publisher 		= nullptr;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr   m_image_small_full_640_kar_publisher 	= nullptr;

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr 	m_depth_publisher					= nullptr;
	
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr 	m_fps_publisher 					= nullptr;

	rclcpp::Subscription<camera_interfaces::msg::DepthFrameset>::SharedPtr m_frameset_subscription;

	void framesetCallback(camera_interfaces::msg::DepthFrameset::UniquePtr fset_msg);
	void publishImage(uint8_t * color_image, int width, int height, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr message_publisher);
	void publishDepthImage(uint16_t * depth_image, int width, int height, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr message_publisher);

	cv::Mat rotateAndScale(cv::cuda::GpuMat& input);
	void blurAndRemoveBackground(cv::cuda::GpuMat& color_input, cv::cuda::GpuMat& depth_input);

	void CheckFPS(uint64_t* pFrameCnt);
	void PrintFPS(const float fps, const float itrTime);

};
