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

	int m_image_rotation, m_depth_thr_min, m_depth_thr_max;
	int m_open_kernel_size, m_close_kernel_size, m_blur_kernel_size, m_black_blur_kernel_size;

	int m_os_image_size = 416;

	uint8_t 	*m_send_color_full_bytes 			= NULL;
	uint8_t 	*m_send_color_limited_bytes 		= NULL;
	uint16_t 	*m_send_depth_full_bytes 			= NULL;
	uint16_t 	*m_send_depth_limited_bytes 		= NULL;
	uint8_t 	*m_send_color_small_limited_bytes 	= NULL;
	//uint8_t 	*m_send_color_ostest_bytes 			= NULL;

	cv::cuda::GpuMat m_color_cuda;
	cv::cuda::GpuMat m_color_cuda_rotated;
	cv::cuda::GpuMat m_color_cuda_flipped;
	cv::cuda::GpuMat m_color_small_cuda_out;
	cv::cuda::GpuMat m_color_ostest_cuda_out;
	cv::cuda::GpuMat m_color_ostest_cuda_out_rgb;

	cv::cuda::GpuMat m_depth_cuda;
	cv::cuda::GpuMat m_depth_cuda_rotated;
	cv::cuda::GpuMat m_depth_cuda_flipped;
	cv::cuda::GpuMat m_depth_cuda_gaus;
	cv::cuda::GpuMat m_depth_cuda_linear;
	cv::cuda::GpuMat m_depth_cuda_thr;
	cv::cuda::GpuMat m_depth_cuda_convU8;
	cv::cuda::GpuMat m_depth_cuda_morph;
	cv::cuda::GpuMat m_depth_cuda_morph_close;

	cv::cuda::GpuMat m_depth_small_cuda_thr;
	cv::cuda::GpuMat m_depth_small_cuda_thr_2;
	cv::cuda::GpuMat m_depth_small_cuda;
	cv::cuda::GpuMat m_depth_small_cuda_convU8;
	cv::cuda::GpuMat m_depth_small_cuda_convU8_2;
	cv::cuda::GpuMat m_depth_small_cuda_morph;
	cv::cuda::GpuMat m_depth_small_cuda_morph_open;
	
	cv::cuda::GpuMat m_depth_cuda_out_convU8;
	cv::cuda::GpuMat m_depth_cuda_out_open;
	cv::cuda::GpuMat m_depth_cuda_out_close;
	cv::cuda::GpuMat color_cuda_out_CV_8UC4;

	cv::Mat m_depth_limited_out;
	cv::Mat m_color_limited_out;
	cv::Mat m_color_small_limited_out;
	cv::Mat m_color_ostest_out;
	cv::Mat m_color_out; 
	cv::Mat m_depth_out;

	rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
	OnSetParametersCallbackHandle::SharedPtr callback_handle_;

	cv::Ptr<cv::cuda::Filter> m_linear_small;
	cv::Ptr<cv::cuda::Filter> m_linear_black;
	cv::Ptr<cv::cuda::Filter> m_morph_filter_open;
	cv::Ptr<cv::cuda::Filter> m_morph_filter_close;
	cv::Ptr<cv::cuda::Filter> m_morph_filter_dilate;

	std::string m_window_name_depth	 			= "Depth_Frame";
	std::string m_window_name_depth_small		= "Depth_Frame_Small";

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr 	m_image_limited_publisher			= nullptr;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr 	m_image_full_publisher		   		= nullptr;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr   m_image_small_full_kar_publisher 	= nullptr;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr 	m_image_small_limited_kar_publisher = nullptr;
	//rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr 	m_image_small_ostest_publisher		= nullptr;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr 	m_depth_publisher					= nullptr;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr 	m_depth_limited_publisher			= nullptr;
	
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr 	m_fps_publisher 					= nullptr;

	rclcpp::Subscription<camera_interfaces::msg::DepthFrameset>::SharedPtr m_frameset_subscription;

	void framesetCallback(camera_interfaces::msg::DepthFrameset::UniquePtr fset_msg);
	void publishImage(uint8_t * color_image, int width, int height, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr message_publisher);
	void publishDepthImage(uint16_t * depth_image, int width, int height, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr message_publisher);

	void CheckFPS(uint64_t* pFrameCnt);
	void PrintFPS(const float fps, const float itrTime);

};
