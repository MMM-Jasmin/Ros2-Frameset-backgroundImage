#include "background_image_node.hpp"
#include "Utils.h"
#include <opencv2/photo.hpp>
#include <opencv2/photo/cuda.hpp>

const double ONE_SECOND            = 1000.0; // One second in milliseconds

/**
 * @brief Contructor.
 */
BackgroundImageNode::BackgroundImageNode() : Node("background_image_node", rclcpp::NodeOptions().use_intra_process_comms(false))
{
	this->declare_parameter("rotation", 0);
	this->declare_parameter("in_topic", "");
	this->declare_parameter("out_limited_topic", "");
	this->declare_parameter("out_full_topic", "");
	this->declare_parameter("out_small_topic", "");
	this->declare_parameter("out_depth_topic", "");
	this->declare_parameter("fps_topic", "test/fps");
	this->declare_parameter("depth_thr_max", 2000);
	this->declare_parameter("depth_thr_min", 500);
	this->declare_parameter("print_fps", true);
	this->declare_parameter("FPS_STR", "FPS" );
	this->declare_parameter("qos_sensor_data", true);
	this->declare_parameter("qos_history_depth", 10);
	this->declare_parameter("open_kernel_size", 19);
	this->declare_parameter("close_kernel_size", 19);
	this->declare_parameter("blur_kernel_size",3);
	this->declare_parameter("black_blur_kernel_size",5);
}

/**
 * @brief Initialize image node.
 */
void BackgroundImageNode::init()
{

	bool qos_sensor_data;
	std::string in_ros_topic, out_ros_topic_limited, out_small_ros_topic, fps_topic, out_ros_topic_full, out_depth_topic;
	int qos_history_depth;

	this->get_parameter("in_topic", in_ros_topic);
	this->get_parameter("out_limited_topic", out_ros_topic_limited);
	this->get_parameter("out_full_topic", out_ros_topic_full);
	this->get_parameter("out_small_topic", out_small_ros_topic);
	this->get_parameter("out_depth_topic", out_depth_topic);
	this->get_parameter("fps_topic", fps_topic);
	this->get_parameter("rotation", m_image_rotation);
	this->get_parameter("depth_thr_max", m_depth_thr_max);
	this->get_parameter("depth_thr_min", m_depth_thr_min);
	this->get_parameter("print_fps", m_print_fps);
	this->get_parameter("FPS_STR", m_FPS_STR );
	this->get_parameter("qos_sensor_data", qos_sensor_data);
	this->get_parameter("qos_history_depth", qos_history_depth);
	this->get_parameter("open_kernel_size", m_open_kernel_size);
	this->get_parameter("close_kernel_size", m_close_kernel_size);
	this->get_parameter("blur_kernel_size", m_blur_kernel_size);
	this->get_parameter("black_blur_kernel_size", m_black_blur_kernel_size);

	rclcpp::QoS m_qos_profile = rclcpp::SensorDataQoS();
	m_qos_profile = m_qos_profile.keep_last(5);
	m_qos_profile = m_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
	m_qos_profile = m_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
	
	rclcpp::QoS m_qos_profile_sysdef = rclcpp::SystemDefaultsQoS();
	m_qos_profile_sysdef = m_qos_profile_sysdef.keep_last(5);
	m_qos_profile_sysdef = m_qos_profile_sysdef.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
	m_qos_profile_sysdef = m_qos_profile_sysdef.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

	m_elapsedTime = 0;
	m_timer.Start();

	//m_depth_subscription = this->create_subscription<sensor_msgs::msg::Image>(m_topic_depth, m_qos_profile, std::bind(&ImageNode::depthCallback, this, std::placeholders::_1));
	m_frameset_subscription = this->create_subscription<camera_interfaces::msg::DepthFrameset>(in_ros_topic, m_qos_profile, std::bind(&BackgroundImageNode::framesetCallback, this, std::placeholders::_1));
	//cv::namedWindow(m_window_name_frameset_0, cv::WINDOW_AUTOSIZE);
	//cv::namedWindow(m_window_name_depth, cv::WINDOW_AUTOSIZE);
	//cv::namedWindow(m_window_name_depth_small,cv::WINDOW_AUTOSIZE);

	cv::Mat open_kernel	= cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(m_open_kernel_size,m_open_kernel_size));
	m_morph_filter_open	= cv::cuda::createMorphologyFilter(cv::MORPH_OPEN, CV_8UC1, open_kernel);

	cv::Mat close_kernel 	= cv::getStructuringElement(cv::MORPH_RECT, cv::Size(m_close_kernel_size, m_close_kernel_size));
	m_morph_filter_close 	= cv::cuda::createMorphologyFilter(cv::MORPH_CLOSE, CV_8UC1, close_kernel);

	// Update kernel size for a normalized box filter
	cv::Mat kernel = cv::Mat::ones( m_blur_kernel_size, m_blur_kernel_size, CV_32F )/ (float)(m_blur_kernel_size*m_blur_kernel_size);
	m_linear_small = cv::cuda::createLinearFilter(CV_16UC1, CV_16UC1, kernel);

	cv::Mat black_kernel = cv::Mat::ones( m_black_blur_kernel_size, m_black_blur_kernel_size, CV_32F )/ (float)(m_black_blur_kernel_size*m_black_blur_kernel_size);
	m_linear_black = cv::cuda::createLinearFilter(CV_8U, CV_8U, black_kernel);
	
	m_image_full_publisher 					= this->create_publisher<sensor_msgs::msg::Image>(out_ros_topic_full, m_qos_profile);
	m_image_limited_publisher 				= this->create_publisher<sensor_msgs::msg::Image>(out_ros_topic_limited, m_qos_profile);	
	m_depth_publisher 						= this->create_publisher<sensor_msgs::msg::Image>(out_depth_topic, m_qos_profile);
	m_depth_limited_publisher 				= this->create_publisher<sensor_msgs::msg::Image>(out_depth_topic + "_limited", m_qos_profile);
	m_image_small_limited_kar_publisher		= this->create_publisher<sensor_msgs::msg::Image>(out_small_ros_topic + "_limited", m_qos_profile);
	//m_image_small_full_kar_publisher		= this->create_publisher<sensor_msgs::msg::Image>(out_small_ros_topic + "_full_640_kar", m_qos_profile);

	m_fps_publisher    						= this->create_publisher<std_msgs::msg::String>(fps_topic, m_qos_profile_sysdef);

	callback_handle_ = this->add_on_set_parameters_callback(std::bind(&BackgroundImageNode::parametersCallback, this, std::placeholders::_1));
}

void BackgroundImageNode::publishImage(uint8_t * color_image, int width, int height, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr message_publisher)
{
	sensor_msgs::msg::Image color_msg;
	color_msg.width           = width;
	color_msg.height          = height;
	color_msg.is_bigendian    = false;
	color_msg.step            = color_msg.width * 3 * sizeof(uint8_t);
	color_msg.encoding        = "rgb8";
	color_msg.data.assign(color_image, color_image + (color_msg.step * color_msg.height));

	sensor_msgs::msg::Image::UniquePtr color_msg_ptr = std::make_unique<sensor_msgs::msg::Image>(color_msg);

	try{
		message_publisher->publish(std::move(color_msg));
	}
  	catch (...) {
    	RCLCPP_INFO(this->get_logger(), "message_publisher: hmm publishing image has failed!! ");
  	}
}

void BackgroundImageNode::publishDepthImage(uint16_t * depth_image, int width, int height, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr message_publisher)
{
		uint8_t * ui8_depth_image = reinterpret_cast<uint8_t *>(depth_image);

		sensor_msgs::msg::Image depth_msg;
		depth_msg.width           = width;
		depth_msg.height          = height;
		depth_msg.is_bigendian    = false;
		depth_msg.step            = depth_msg.width * sizeof(uint16_t);
		depth_msg.encoding        = "mono16";
		depth_msg.data.assign(ui8_depth_image, ui8_depth_image + (depth_msg.step * depth_msg.height));

		sensor_msgs::msg::Image::UniquePtr depth_msg_ptr = std::make_unique<sensor_msgs::msg::Image>(depth_msg);

	try{
		message_publisher->publish(std::move(depth_msg_ptr));
	}
  	catch (...) {
    	RCLCPP_INFO(this->get_logger(), "message_publisher: hmm publishing depth image has failed!! ");
  	}

}

rcl_interfaces::msg::SetParametersResult BackgroundImageNode::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
{
	for (const auto &param: parameters){
		if (param.get_name() == "depth_thr_max"){
			m_depth_thr_max = param.as_int();
		} else if (param.get_name() == "open_kernel_size"){
			m_open_kernel_size = param.as_int();
			cv::Mat open_kernel	= cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(param.as_int(),param.as_int()));
			m_morph_filter_open  	= cv::cuda::createMorphologyFilter(cv::MORPH_OPEN, CV_8UC1, open_kernel);
		} else if (param.get_name() == "close_kernel_size"){
			m_close_kernel_size = param.as_int();
			cv::Mat close_kernel 	= cv::getStructuringElement(cv::MORPH_RECT, cv::Size(param.as_int(), param.as_int()));
			m_morph_filter_close 	= cv::cuda::createMorphologyFilter(cv::MORPH_CLOSE, CV_8UC1, close_kernel);
		} else if (param.get_name() == "blur_kernel_size"){
			m_blur_kernel_size = param.as_int();
			cv::Mat kernel = cv::Mat::ones( param.as_int(), param.as_int(), CV_32F )/ (float)(param.as_int()*param.as_int());
			m_linear_small = cv::cuda::createLinearFilter(CV_16UC1, CV_16UC1, kernel);
		} else if (param.get_name() == "black_blur_kernel_size"){
			m_black_blur_kernel_size = param.as_int();
			cv::Mat kernel = cv::Mat::ones( param.as_int(), param.as_int(), CV_32F )/ (float)(param.as_int()*param.as_int());
			m_linear_black = cv::cuda::createLinearFilter(CV_8U, CV_8U, kernel);
		}
	}

	rcl_interfaces::msg::SetParametersResult result;
	result.successful = true;
    result.reason = "success";
	return result;
}


/**
 * @brief Callback function for reveived image message.
 * @param img_msg Received image message
 */
void BackgroundImageNode::framesetCallback(camera_interfaces::msg::DepthFrameset::UniquePtr fset_msg)
{
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

	int image_width = static_cast<int>(fset_msg.get()->color_image.width);
	int image_height = static_cast<int>(fset_msg.get()->color_image.height);
	int send_width, send_heigth;
	
	if(m_image_rotation == 90 || m_image_rotation == 270){
		send_width = image_height;
		send_heigth = image_width;
	}else{
		send_width = image_width;
		send_heigth = image_height;
	}

	int max_wh = std::max(image_width, image_height);
	float scaleFactorKAR = 640.0 / max_wh;  // resize image to img_size

	int KAR_640_width = int(send_width * scaleFactorKAR);
	int KAR_640_height = int(send_heigth * scaleFactorKAR);

	//std::cout << KAR_640_width << " " << KAR_640_height << std::endl;

	if (m_send_color_limited_bytes == NULL)
		m_send_color_limited_bytes  = reinterpret_cast<uint8_t*>(malloc(static_cast<unsigned>(image_width * image_height) * 3 * sizeof(uint8_t)));

	if (m_send_color_full_bytes == NULL)
		m_send_color_full_bytes  = reinterpret_cast<uint8_t*>(malloc(static_cast<unsigned>(image_width * image_height) *3 * sizeof(uint8_t)));

	if (m_send_color_small_limited_bytes == NULL)
		m_send_color_small_limited_bytes = reinterpret_cast<uint8_t *>(malloc(KAR_640_width * KAR_640_height * 3 * sizeof(uint8_t)));

	if (m_send_depth_full_bytes == NULL)
		m_send_depth_full_bytes  = reinterpret_cast<uint16_t *>(malloc(image_width * image_height * sizeof(uint16_t)));

	if (m_send_depth_limited_bytes == NULL)
		m_send_depth_limited_bytes  = reinterpret_cast<uint16_t *>(malloc(image_width * image_height * sizeof(uint16_t)));


	auto future_publishImage_full			= std::async(&BackgroundImageNode::publishImage, 		this, m_send_color_full_bytes, 			send_width, 	send_heigth, 	m_image_full_publisher);
	auto future_publishImage_limited		= std::async(&BackgroundImageNode::publishImage, 		this, m_send_color_limited_bytes,		send_width,	 	send_heigth, 	m_image_limited_publisher);
	auto future_publishDepthImage			= std::async(&BackgroundImageNode::publishDepthImage, 	this, m_send_depth_full_bytes, 			send_width, 	send_heigth, 	m_depth_publisher);
	auto future_publishDepthImage_limited	= std::async(&BackgroundImageNode::publishDepthImage,	this, m_send_depth_limited_bytes, 		send_width, 	send_heigth, 	m_depth_limited_publisher);
	auto future_publishImage_small_limited	= std::async(&BackgroundImageNode::publishImage, 		this, m_send_color_small_limited_bytes, KAR_640_width, 	KAR_640_height, m_image_small_limited_kar_publisher);
	
	cv::Size image_size(image_width, image_height);
	cv::Mat color_image(image_size, CV_8UC3, (void *)fset_msg.get()->color_image.data.data(), cv::Mat::AUTO_STEP);
	cv::Mat depth_image(image_size, CV_16UC1, (void *)fset_msg.get()->depth_image.data.data(), cv::Mat::AUTO_STEP);
	

	
	m_depth_cuda.upload(depth_image);
	m_color_cuda.upload(color_image);

	if (m_image_rotation == 90) {
		cv::cuda::rotate(m_color_cuda, m_color_cuda_rotated, cv::Size( m_color_cuda.size().height, m_color_cuda.size().width ), 270, m_color_cuda.size().height-1, 0, cv::INTER_LINEAR  );
		cv::cuda::flip(m_color_cuda_rotated, m_color_cuda_flipped, 1);

		cv::cuda::rotate(m_depth_cuda, m_depth_cuda_rotated, cv::Size( m_depth_cuda.size().height, m_depth_cuda.size().width ), 270, m_depth_cuda.size().height-1, 0, cv::INTER_LINEAR  );
		cv::cuda::flip(m_depth_cuda_rotated, m_depth_cuda_flipped, 1);

	} else if (m_image_rotation == 180) {
		cv::cuda::rotate(m_color_cuda, m_color_cuda_rotated, cv::Size( m_color_cuda.size().width, m_color_cuda.size().height ), 180, m_color_cuda.size().width -1, m_color_cuda.size().height-1, cv::INTER_LINEAR  );
		cv::cuda::flip(m_color_cuda_rotated, m_color_cuda_flipped, 1);

		cv::cuda::rotate(m_depth_cuda, m_depth_cuda_rotated, cv::Size( m_depth_cuda.size().width, m_depth_cuda.size().height ), 180, m_depth_cuda.size().width -1, m_depth_cuda.size().height-1, cv::INTER_LINEAR  );
		cv::cuda::flip(m_depth_cuda_rotated, m_depth_cuda_flipped, 1);

	} else if (m_image_rotation == 270) {
		cv::cuda::rotate(m_color_cuda, m_color_cuda_rotated, cv::Size( m_color_cuda.size().height, m_color_cuda.size().width ), 90, 0, m_color_cuda.size().width -1, cv::INTER_LINEAR  );
		cv::cuda::flip(m_color_cuda_rotated, m_color_cuda_flipped, 1);

		cv::cuda::rotate(m_depth_cuda, m_depth_cuda_rotated, cv::Size( m_depth_cuda.size().height, m_depth_cuda.size().width ), 90, 0, m_depth_cuda.size().width -1, cv::INTER_LINEAR  );
		cv::cuda::flip(m_depth_cuda_rotated, m_depth_cuda_flipped, 1);

	} else {
		cv::cuda::flip(m_color_cuda, m_color_cuda_flipped, 1);
		cv::cuda::flip(m_depth_cuda, m_depth_cuda_flipped, 1);
	}

	m_color_cuda_flipped.download(m_color_out);
	m_depth_cuda_flipped.download(m_depth_out);
	
	m_linear_small->apply(m_depth_cuda_flipped, m_depth_cuda_linear);
	
	cv::cuda::threshold(m_depth_cuda_linear, m_depth_cuda_thr, m_depth_thr_min, 0 , cv::THRESH_TOZERO);
	cv::cuda::threshold(m_depth_cuda_linear, m_depth_cuda_thr, m_depth_thr_max, 255 , cv::THRESH_TOZERO_INV);

	cv::cuda::resize(m_depth_cuda_thr, m_depth_small_cuda, cv::Size(272,480), cv::INTER_AREA);

	m_depth_small_cuda.convertTo(m_depth_small_cuda_convU8, CV_8UC1);

	m_morph_filter_open->apply(m_depth_small_cuda_convU8, m_depth_small_cuda_morph_open);
	m_morph_filter_close->apply(m_depth_small_cuda_morph_open, m_depth_small_cuda_morph);
	
	cv::cuda::resize(m_depth_small_cuda_morph, m_depth_cuda_morph, cv::Size(m_depth_cuda_flipped.size().width, m_depth_cuda_flipped.size().height), cv::INTER_LINEAR);

	cv::cuda::GpuMat depth_cuda_out;
	cv::cuda::GpuMat color_cuda_out;
	cv::cuda::GpuMat depth_cuda_out_tmp;

	m_depth_cuda_thr.copyTo(depth_cuda_out, m_depth_cuda_morph);
	depth_cuda_out.download(m_depth_limited_out);

	depth_cuda_out.convertTo(m_depth_cuda_convU8, CV_8UC1);

	m_color_cuda_flipped.copyTo(color_cuda_out, m_depth_cuda_convU8);

	cv::cuda::cvtColor(color_cuda_out,color_cuda_out_CV_8UC4,cv::COLOR_BGR2GRAY);
 	m_linear_black->apply(color_cuda_out_CV_8UC4, color_cuda_out_CV_8UC4);
	cv::cuda::cvtColor(color_cuda_out_CV_8UC4, color_cuda_out,cv::COLOR_GRAY2BGR);

	m_color_cuda_flipped.copyTo(color_cuda_out, m_depth_cuda_convU8);
	color_cuda_out.download(m_color_limited_out);

	cv::cuda::resize(color_cuda_out, m_color_small_cuda_out, cv::Size(KAR_640_width,KAR_640_height), cv::INTER_AREA);

	m_color_small_cuda_out.download(m_color_small_limited_out);

	future_publishImage_full.wait();
	future_publishImage_limited.wait();
	future_publishDepthImage.wait();
	future_publishDepthImage_limited.wait();
	future_publishImage_small_limited.wait();

	std::memcpy(reinterpret_cast<void*>(m_send_color_full_bytes), m_color_out.data, m_color_out.size().width * m_color_out.size().height * 3 * sizeof(uint8_t));
	std::memcpy(reinterpret_cast<void*>(m_send_color_limited_bytes), m_color_limited_out.data, m_color_limited_out.size().width * m_color_limited_out.size().height * 3 * sizeof(uint8_t));
	std::memcpy(reinterpret_cast<void*>(m_send_depth_full_bytes), m_depth_out.data, m_depth_out.size().width * m_depth_out.size().height * sizeof(uint16_t));
	std::memcpy(reinterpret_cast<void*>(m_send_depth_limited_bytes), m_depth_limited_out.data, m_depth_limited_out.size().width * m_depth_limited_out.size().height * sizeof(uint16_t));
	std::memcpy(reinterpret_cast<void*>(m_send_color_small_limited_bytes), 	m_color_small_limited_out.data,	m_color_small_limited_out.size().width * m_color_small_limited_out.size().height * 3 * sizeof(uint8_t));
	
	m_frameCnt++;
	CheckFPS(&m_frameCnt);

}

void BackgroundImageNode::CheckFPS(uint64_t* pFrameCnt)
	{
		m_timer.Stop();

		double itrTime      = m_timer.GetElapsedTimeInMilliSec();
		double fps;

		m_elapsedTime += itrTime;

		fps = 1000 / (m_elapsedTime / (*pFrameCnt));

		if (m_elapsedTime >= ONE_SECOND)
		{
			PrintFPS(fps, itrTime);

			*pFrameCnt    = 0;
			m_elapsedTime = 0;
		}

		m_timer.Start();
	}

void BackgroundImageNode::PrintFPS(const float fps, const float itrTime)
{
		
	std::stringstream str("");

	if (fps == 0.0f)
			str << string_format("{\"%s\": 0.0}", m_FPS_STR.c_str());
	else
		str << string_format("{\"%s\": %.2f, \"lastCurrMSec\": %.2f }", m_FPS_STR.c_str(), fps, itrTime);

	auto message = std_msgs::msg::String();
	message.data = str.str();
	
	try{
		m_fps_publisher->publish(message);
	}
  	catch (...) {
    	RCLCPP_INFO(this->get_logger(), "m_fps_publisher: hmm publishing dets has failed!! ");
  	}

	if (m_print_fps)
		RCLCPP_INFO(this->get_logger(), message.data.c_str());
}
