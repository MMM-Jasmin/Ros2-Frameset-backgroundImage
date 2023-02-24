#include "background_image_node.hpp"
#include "Utils.h"

const double ONE_SECOND            = 1000.0; // One second in milliseconds

/**
 * @brief Contructor.
 */
BackgroundImageNode::BackgroundImageNode() : Node("background_image_node", rclcpp::NodeOptions().use_intra_process_comms(false))
{
	this->declare_parameter("rotation", 0);
	this->declare_parameter("in_topic", "");
	this->declare_parameter("out_topic", "");
	this->declare_parameter("out_full_topic", "");
	this->declare_parameter("out_small_topic", "");
	this->declare_parameter("out_depth_topic", "");
	this->declare_parameter("fps_topic", "test/fps");
	this->declare_parameter("max_fps", 30.0f);
	this->declare_parameter("depth_thr", 0);
	this->declare_parameter("print_fps", true);
	this->declare_parameter("FPS_STR", "FPS" );
	this->declare_parameter("qos_sensor_data", true);
	this->declare_parameter("qos_history_depth", 10);
}

/**
 * @brief Initialize image node.
 */
void BackgroundImageNode::init()
{

	bool qos_sensor_data;
	std::string in_ros_topic, out_ros_topic, out_small_ros_topic, fps_topic, out_ros_topic_full, out_depth_topic;
	int qos_history_depth;

	this->get_parameter("in_topic", in_ros_topic);
	this->get_parameter("out_topic", out_ros_topic);
	this->get_parameter("out_full_topic", out_ros_topic_full);
	this->get_parameter("out_small_topic", out_small_ros_topic);
	this->get_parameter("out_depth_topic", out_depth_topic);
	this->get_parameter("fps_topic", fps_topic);
	this->get_parameter("max_fps", m_maxFPS);
	this->get_parameter("rotation", m_image_rotation);
	this->get_parameter("depth_thr", m_depth_thr);
	this->get_parameter("print_fps", m_print_fps);
	this->get_parameter("FPS_STR", m_FPS_STR );
	this->get_parameter("qos_sensor_data", qos_sensor_data);
	this->get_parameter("qos_history_depth", qos_history_depth);

	//rclcpp::QoS m_qos_profile = rclcpp::SystemDefaultsQoS();
	//rclcpp::QoS m_qos_profile_sysdef = rclcpp::SystemDefaultsQoS();

	//m_qos_profile = m_qos_profile.keep_last(qos_history_depth);
	//m_qos_profile = m_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
	//m_qos_profile = m_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
	//
	//m_qos_profile_sysdef = m_qos_profile_sysdef.keep_last(qos_history_depth);
	//m_qos_profile_sysdef = m_qos_profile_sysdef.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
	//m_qos_profile_sysdef = m_qos_profile_sysdef.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);


	rclcpp::QoS m_qos_profile = rclcpp::SensorDataQoS();
	m_qos_profile = m_qos_profile.keep_last(5);
	m_qos_profile = m_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
	m_qos_profile = m_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
	
	rclcpp::QoS m_qos_profile_sysdef = rclcpp::SystemDefaultsQoS();
	m_qos_profile_sysdef = m_qos_profile_sysdef.keep_last(5);
	m_qos_profile_sysdef = m_qos_profile_sysdef.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
	m_qos_profile_sysdef = m_qos_profile_sysdef.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

	//rclcpp::QoS m_qos_profile_BEST = m_qos_profile_sysdef.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
	
	//m_qos_profile_sysdef = m_qos_profile_sysdef.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

	//m_qos_profile_sysdef = m_qos_profile_sysdef.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
	//m_qos_profile = m_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
	
	//m_qos_profile = m_qos_profile.deadline(std::chrono::nanoseconds(static_cast<int>(1e9 / 30)));
	
	m_elapsedTime = 0;
	m_timer.Start();

	//m_depth_subscription = this->create_subscription<sensor_msgs::msg::Image>(m_topic_depth, m_qos_profile, std::bind(&ImageNode::depthCallback, this, std::placeholders::_1));
	m_frameset_subscription = this->create_subscription<camera_interfaces::msg::DepthFrameset>(in_ros_topic, m_qos_profile, std::bind(&BackgroundImageNode::framesetCallback, this, std::placeholders::_1));
	//cv::namedWindow(m_window_name_frameset_0, cv::WINDOW_AUTOSIZE);

	//m_gaussian = cv::cuda::createGaussianFilter(depth_image.type(),depth_image.type(), Size(31,31),0);
	m_gaussian = cv::cuda::createGaussianFilter(CV_16UC1,CV_8UC1, cv::Size(3,3),1);

	m_image_publisher 					= this->create_publisher<sensor_msgs::msg::Image>(out_ros_topic, m_qos_profile);
	m_image_full_publisher 				= this->create_publisher<sensor_msgs::msg::Image>(out_ros_topic_full, m_qos_profile);
	m_image_small_416_publisher 		= this->create_publisher<sensor_msgs::msg::Image>(out_small_ros_topic + "_416", m_qos_profile);
	m_image_small_608_publisher 		= this->create_publisher<sensor_msgs::msg::Image>(out_small_ros_topic + "_608", m_qos_profile);
	m_image_small_640_publisher 		= this->create_publisher<sensor_msgs::msg::Image>(out_small_ros_topic + "_640", m_qos_profile);
	m_image_small_full_416_publisher 	= this->create_publisher<sensor_msgs::msg::Image>(out_small_ros_topic + "_full_416", m_qos_profile);
	m_image_small_full_608_publisher 	= this->create_publisher<sensor_msgs::msg::Image>(out_small_ros_topic + "_full_608", m_qos_profile);

	m_depth_publisher 					= this->create_publisher<sensor_msgs::msg::Image>(out_depth_topic, m_qos_profile);

	m_fps_publisher    					= this->create_publisher<std_msgs::msg::String>(fps_topic, m_qos_profile_sysdef);

	callback_handle_ = this->add_on_set_parameters_callback(std::bind(&BackgroundImageNode::parametersCallback, this, std::placeholders::_1));

	m_send_frame_small_416_bytes  		= reinterpret_cast<uint8_t 	*>(malloc(static_cast<unsigned>(416 * 416) * 3 * sizeof(uint8_t)));
	m_send_frame_small_608_bytes  		= reinterpret_cast<uint8_t 	*>(malloc(static_cast<unsigned>(608 * 608) * 3 * sizeof(uint8_t)));
	m_send_frame_small_640_bytes  		= reinterpret_cast<uint8_t 	*>(malloc(static_cast<unsigned>(640 * 640) * 3 * sizeof(uint8_t)));
	m_send_frame_small_full_416_bytes  	= reinterpret_cast<uint8_t 	*>(malloc(static_cast<unsigned>(416 * 416) * 3 * sizeof(uint8_t)));
	m_send_frame_small_full_608_bytes  	= reinterpret_cast<uint8_t 	*>(malloc(static_cast<unsigned>(608 * 608) * 3 * sizeof(uint8_t)));


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
		if (param.get_name() == "depth_thr")
			m_depth_thr = param.as_int();
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

	int image_width = static_cast<int>(fset_msg.get()->color_image.width);
	int image_height = static_cast<int>(fset_msg.get()->color_image.height);
	int send_width, send_heigth;

	if (m_send_frame_bytes == NULL)
		m_send_frame_bytes  	 = reinterpret_cast<uint8_t*>(malloc(static_cast<unsigned>(image_width * image_height) * 3 * sizeof(uint8_t)));

	if (m_send_frame_full_bytes == NULL)
		m_send_frame_full_bytes  = reinterpret_cast<uint8_t*>(malloc(static_cast<unsigned>(image_width * image_height) * 3 * sizeof(uint8_t)));

	if (m_send_depth_frame_bytes == NULL)
		m_send_depth_frame_bytes  = reinterpret_cast<uint16_t *>(malloc(image_width * image_height * sizeof(uint16_t)));

	if(m_image_rotation == 90 || m_image_rotation == 270){
		send_width = image_height;
		send_heigth = image_width;
	}else{
		send_width = image_width;
		send_heigth = image_height;
	}
				
	auto future_publishImage 			= std::async(&BackgroundImageNode::publishImage, this, m_send_frame_bytes, send_width, send_heigth, m_image_publisher);
	auto future_publishImage_full		= std::async(&BackgroundImageNode::publishImage, this, m_send_frame_full_bytes, send_width, send_heigth, m_image_full_publisher);
	auto future_publishImage_416 		= std::async(&BackgroundImageNode::publishImage, this, m_send_frame_small_416_bytes, 416, 416, m_image_small_416_publisher);
	auto future_publishImage_608 		= std::async(&BackgroundImageNode::publishImage, this, m_send_frame_small_608_bytes, 608, 608, m_image_small_608_publisher);
	auto future_publishImage_640 		= std::async(&BackgroundImageNode::publishImage, this, m_send_frame_small_640_bytes, 640, 640, m_image_small_640_publisher);		
	auto future_publishImage_full_416 	= std::async(&BackgroundImageNode::publishImage, this, m_send_frame_small_full_416_bytes, 416, 416, m_image_small_full_416_publisher);
	auto future_publishImage_full_608 	= std::async(&BackgroundImageNode::publishImage, this, m_send_frame_small_full_608_bytes, 608, 608, m_image_small_full_608_publisher);
	auto future_publishDepthImage 		= std::async(&BackgroundImageNode::publishDepthImage, this, m_send_depth_frame_bytes, send_width, send_heigth, m_depth_publisher);
		
	cv::Size image_size(image_width, image_height);
	cv::Mat color_image(image_size, CV_8UC3, (void *)fset_msg.get()->color_image.data.data(), cv::Mat::AUTO_STEP);
	cv::Mat depth_image(image_size, CV_16UC1, (void *)fset_msg.get()->depth_image.data.data(), cv::Mat::AUTO_STEP);

	cv::Mat color_image_original;
	cv::Mat color_image_small_416;
	cv::Mat color_image_small_608;
	cv::Mat color_image_small_640;
	cv::Mat color_image_small_full_416;
	cv::Mat color_image_small_full_608;

	cv::Mat depth_image_original;

	cv::cuda::GpuMat depth_image_cuda;	
	cv::cuda::GpuMat depth_image_cuda_tmp;
	cv::cuda::GpuMat color_image_cuda;
	cv::cuda::GpuMat color_image_cuda_tmp;
	cv::cuda::GpuMat color_image_cuda_out;
	cv::cuda::GpuMat color_image_cuda_small_416;
	cv::cuda::GpuMat color_image_cuda_small_608;
	cv::cuda::GpuMat color_image_cuda_small_640;
	cv::cuda::GpuMat color_image_cuda_small_full_416;
	cv::cuda::GpuMat color_image_cuda_small_full_608;

	depth_image_cuda.upload(depth_image);
	color_image_cuda.upload(color_image);

	
	if (m_image_rotation == 90) {
		cv::cuda::rotate( color_image_cuda, color_image_cuda_tmp, cv::Size( color_image_cuda.size().height, color_image_cuda.size().width ), 270, color_image_cuda.size().height-1, 0, cv::INTER_LINEAR  );
		cv::cuda::flip(color_image_cuda_tmp, color_image_cuda, 1);

		cv::cuda::rotate( depth_image_cuda, depth_image_cuda_tmp, cv::Size( depth_image_cuda.size().height, depth_image_cuda.size().width ), 270, depth_image_cuda.size().height-1, 0, cv::INTER_LINEAR  );
		cv::cuda::flip(depth_image_cuda_tmp, depth_image_cuda, 1);

	} else if (m_image_rotation == 180) {
		cv::cuda::rotate(color_image_cuda, color_image_cuda_tmp, cv::Size( color_image_cuda.size().width, color_image_cuda.size().height ), 180, color_image_cuda.size().width -1, color_image_cuda.size().height-1, cv::INTER_LINEAR  );
		cv::cuda::flip(color_image_cuda_tmp, color_image_cuda, 1);

		cv::cuda::rotate(depth_image_cuda, depth_image_cuda_tmp, cv::Size( depth_image_cuda.size().width, depth_image_cuda.size().height ), 180, depth_image_cuda.size().width -1, depth_image_cuda.size().height-1, cv::INTER_LINEAR  );
		cv::cuda::flip(depth_image_cuda_tmp, depth_image_cuda, 1);

	} else if (m_image_rotation == 270) {
		cv::cuda::rotate( color_image_cuda, color_image_cuda_tmp, cv::Size( color_image_cuda.size().height, color_image_cuda.size().width ), 90, 0, color_image_cuda.size().width -1, cv::INTER_LINEAR  );
		cv::cuda::flip(color_image_cuda_tmp, color_image_cuda, 1);

		cv::cuda::rotate( depth_image_cuda, depth_image_cuda_tmp, cv::Size( depth_image_cuda.size().height, depth_image_cuda.size().width ), 90, 0, depth_image_cuda.size().width -1, cv::INTER_LINEAR  );
		cv::cuda::flip(depth_image_cuda_tmp, depth_image_cuda, 1);

	} else {
		cv::cuda::flip(color_image_cuda, color_image_cuda_tmp, 1);
		cv::cuda::flip(depth_image_cuda, depth_image_cuda_tmp, 1);
		color_image_cuda = color_image_cuda_tmp;
		depth_image_cuda = depth_image_cuda_tmp;
	}

	color_image_cuda.download(color_image_original);
	depth_image_cuda.download(depth_image_original);
	
	cv::cuda::threshold(depth_image_cuda, depth_image_cuda_tmp, m_depth_thr, 40 , cv::THRESH_TOZERO_INV);
	//cv::cuda::threshold(depth_image_cuda_tmp, depth_image_cuda, 10, 10, cv::THRESH_BINARY); 

	//m_gaussian->apply(depth_image_cuda_tmp, depth_image_cuda);
	
	//cv::cuda::cvtColor(depth_image_cuda_tmp,depth_image_cuda, cv::CV_8UC1
	depth_image_cuda_tmp.convertTo(depth_image_cuda,CV_8UC1);

	//cv::cuda::threshold(depth_image_cuda, depth_image_cuda_tmp, 50, 1, cv::THRESH_BINARY);
	//cv::cuda::threshold(depth_image_cuda_tmp, depth_image_cuda, m_depth_thr, 1, cv::THRESH_OTSU); //cv::THRESH_BINARY);
	
	/**
	
	m_gaussian->apply(depth_image_cuda, depth_image_cuda_tmp);
	
	cv::cuda::threshold(depth_image_cuda_tmp, depth_image_cuda, m_depth_thr, 10, cv::THRESH_TOZERO_INV);
	
	 */
	color_image_cuda.copyTo(color_image_cuda_out, depth_image_cuda);
	color_image_cuda_out.download(color_image);

	cv::cuda::resize(color_image_cuda_out, color_image_cuda_small_416, cv::Size(416,416));
	cv::cuda::resize(color_image_cuda_out, color_image_cuda_small_608, cv::Size(608,608));
	cv::cuda::resize(color_image_cuda_out, color_image_cuda_small_640, cv::Size(640,640));
	cv::cuda::resize(color_image_cuda, color_image_cuda_small_full_416, cv::Size(416,416));
	cv::cuda::resize(color_image_cuda, color_image_cuda_small_full_608, cv::Size(608,608));

	color_image_cuda_small_416.download(color_image_small_416);
	color_image_cuda_small_608.download(color_image_small_608);
	color_image_cuda_small_640.download(color_image_small_640);
	color_image_cuda_small_full_416.download(color_image_small_full_416);
	color_image_cuda_small_full_608.download(color_image_small_full_608);

	future_publishImage.wait();
	future_publishImage_full.wait();
	future_publishImage_416.wait();
	future_publishImage_608.wait();
	future_publishImage_640.wait();
	future_publishImage_full_416.wait();
	future_publishImage_full_608.wait();
	future_publishDepthImage.wait();
	
	std::memcpy(reinterpret_cast<void*>(m_send_frame_bytes), 				color_image.data, 					color_image.size().width * color_image.size().height * 3 * sizeof(uint8_t));
	std::memcpy(reinterpret_cast<void*>(m_send_frame_full_bytes), 			color_image_original.data, 			color_image_original.size().width * color_image_original.size().height * 3 * sizeof(uint8_t));
	std::memcpy(reinterpret_cast<void*>(m_send_frame_small_416_bytes), 		color_image_small_416.data, 		color_image_small_416.size().width * color_image_small_416.size().height * 3 * sizeof(uint8_t));
	std::memcpy(reinterpret_cast<void*>(m_send_frame_small_608_bytes), 		color_image_small_608.data, 		color_image_small_608.size().width * color_image_small_608.size().height * 3 * sizeof(uint8_t));
	std::memcpy(reinterpret_cast<void*>(m_send_frame_small_640_bytes), 		color_image_small_640.data, 		color_image_small_640.size().width * color_image_small_640.size().height * 3 * sizeof(uint8_t) );
	std::memcpy(reinterpret_cast<void*>(m_send_frame_small_full_416_bytes), color_image_small_full_416.data, 	color_image_small_full_416.size().width * color_image_small_full_416.size().height * 3 * sizeof(uint8_t));
	std::memcpy(reinterpret_cast<void*>(m_send_frame_small_full_608_bytes), color_image_small_full_608.data, 	color_image_small_full_608.size().width * color_image_small_full_608.size().height * 3 * sizeof(uint8_t));
	std::memcpy(reinterpret_cast<void*>(m_send_depth_frame_bytes), 			depth_image_original.data, 			depth_image_original.size().width * depth_image_original.size().height * sizeof(uint16_t));
	
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


cv::Mat BackgroundImageNode::rotateAndScale(cv::cuda::GpuMat& color_image_cuda){

	cv::cuda::GpuMat color_image_cuda_tmp;
	cv::Mat color_image;



	return color_image;

	//std::memcpy(reinterpret_cast<void*>(m_send_frame_bytes), color_image.data, m_out_width * m_out_height * 3 * sizeof(uint8_t) );
	//auto future_publishImage = std::async(&BackgroundImageNode::publishImage, this, color_image.data, m_out_width, m_out_height, m_image_publisher);


}
