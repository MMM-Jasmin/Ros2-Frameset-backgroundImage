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
	std::string in_ros_topic, out_ros_topic;
	int qos_history_depth;

	this->get_parameter("in_topic", in_ros_topic);
	this->get_parameter("out_topic", out_ros_topic);
	this->get_parameter("rotation", m_image_rotation);
	this->get_parameter("depth_thr", m_depth_thr);
	this->get_parameter("print_fps", m_print_fps);
	this->get_parameter("FPS_STR", m_FPS_STR );
	this->get_parameter("qos_sensor_data", qos_sensor_data);
	this->get_parameter("qos_history_depth", qos_history_depth);

	if(qos_sensor_data){
		std::cout << "using ROS2 qos_sensor_data" << std::endl;
		m_qos_profile = rclcpp::SensorDataQoS();
	}

	m_qos_profile = m_qos_profile.keep_last(qos_history_depth);
	m_qos_profile = m_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
	m_qos_profile = m_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
	m_qos_profile = m_qos_profile.deadline(std::chrono::nanoseconds(static_cast<int>(1e9 / 30)));
	
	m_elapsedTime = 0;
	m_timer.Start();

	//m_depth_subscription = this->create_subscription<sensor_msgs::msg::Image>(m_topic_depth, m_qos_profile, std::bind(&ImageNode::depthCallback, this, std::placeholders::_1));
	m_frameset_subscription = this->create_subscription<camera_interfaces::msg::DepthFrameset>(in_ros_topic, m_qos_profile, std::bind(&BackgroundImageNode::framesetCallback, this, std::placeholders::_1));
	//cv::namedWindow(m_window_name_frameset_0, cv::WINDOW_AUTOSIZE);

	//m_gaussian = cv::cuda::createGaussianFilter(depth_image.type(),depth_image.type(), Size(31,31),0);
	m_gaussian = cv::cuda::createGaussianFilter(CV_16UC1,CV_8UC1, cv::Size(11,11),1);

	m_image_publisher = this->create_publisher<sensor_msgs::msg::Image>(out_ros_topic, m_qos_profile);

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

	message_publisher->publish(std::move(color_msg));
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

	if (m_send_frame_bytes == NULL)
		m_send_frame_bytes  = reinterpret_cast<uint8_t 	*>(malloc(static_cast<unsigned>(image_width * image_height) * 3 * sizeof(uint8_t)));
	else if(m_image_rotation == 90 || m_image_rotation == 270)
		auto future_publishImage = std::async(&BackgroundImageNode::publishImage, this, m_send_frame_bytes, image_height,image_width, m_image_publisher);
	else
		auto future_publishImage = std::async(&BackgroundImageNode::publishImage, this, m_send_frame_bytes, image_width, image_height, m_image_publisher);

	cv::Size image_size(image_width, image_height);
	cv::Mat color_image(image_size, CV_8UC3, (void *)fset_msg.get()->color_image.data.data(), cv::Mat::AUTO_STEP);
	cv::Mat depth_image(image_size, CV_16UC1, (void *)fset_msg.get()->depth_image.data.data(), cv::Mat::AUTO_STEP);

	cv::cuda::GpuMat depth_image_cuda;	
	cv::cuda::GpuMat depth_image_cuda_tmp;
	cv::cuda::GpuMat color_image_cuda;
	cv::cuda::GpuMat color_image_cuda_tmp;

	depth_image_cuda.upload(depth_image);
	color_image_cuda.upload(color_image);

	cv::cuda::threshold(depth_image_cuda, depth_image_cuda_tmp, m_depth_thr, 100 , cv::THRESH_TOZERO_INV);
	//cv::cuda::threshold(depth_image_cuda_tmp, depth_image_cuda, 10, 10, cv::THRESH_BINARY); 

	m_gaussian->apply(depth_image_cuda_tmp, depth_image_cuda);

	cv::cuda::threshold(depth_image_cuda, depth_image_cuda_tmp, 4, 1, cv::THRESH_BINARY);
	//cv::cuda::threshold(depth_image_cuda_tmp, depth_image_cuda, m_depth_thr, 1, cv::THRESH_OTSU); //cv::THRESH_BINARY);
	
	color_image_cuda.copyTo(color_image_cuda_tmp, depth_image_cuda_tmp);
	
	if (m_image_rotation == 90) {
		cv::cuda::rotate( color_image_cuda_tmp, color_image_cuda, cv::Size( color_image_cuda.size().height, color_image_cuda.size().width ), -90, color_image_cuda.size().height-1, 0, cv::INTER_LINEAR  );
		color_image_cuda.download(color_image);
	} else if (m_image_rotation == 180) {
		cv::cuda::rotate(color_image_cuda_tmp, color_image_cuda, cv::Size( color_image_cuda.size().width, color_image_cuda.size().height ), 180, color_image_cuda.size().width -1, color_image_cuda.size().height-1, cv::INTER_LINEAR  );
		color_image_cuda.download(color_image);
	} else if (m_image_rotation == 270) {
		cv::cuda::rotate( color_image_cuda_tmp, color_image_cuda, cv::Size( color_image_cuda.size().height, color_image_cuda.size().width ), 90, 0, color_image_cuda.size().width -1, cv::INTER_LINEAR  );
		color_image_cuda.download(color_image);
	} else {
		color_image_cuda_tmp.download(color_image);
	}

	std::memcpy(reinterpret_cast<void*>(m_send_frame_bytes), color_image.data, color_image.size().width * color_image.size().height * 3 * sizeof(uint8_t) );

	m_frameCnt++;
	CheckFPS(&m_frameCnt);

	//future_publishImage.get();

	//publishImage(color_image.data, m_out_width, m_out_height, m_image_publisher);

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
	//m_fps_publisher->publish(message);

		
	if (m_print_fps)
		RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

}


cv::Mat BackgroundImageNode::rotateAndScale(cv::cuda::GpuMat& color_image_cuda){

	cv::cuda::GpuMat color_image_cuda_tmp;
	cv::Mat color_image;



	return color_image;

	//std::memcpy(reinterpret_cast<void*>(m_send_frame_bytes), color_image.data, m_out_width * m_out_height * 3 * sizeof(uint8_t) );
	//auto future_publishImage = std::async(&BackgroundImageNode::publishImage, this, color_image.data, m_out_width, m_out_height, m_image_publisher);


}