#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


int main(int argc, char *argv[]){

	int MAX_KERNEL_LENGTH = 31, delay = 500;
	ros::init(argc, argv, "filters");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	
	cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
	cv::Mat dst;

	cv::imshow("Original", image);

	for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 ){
		cv::blur(image, dst, cv::Size(i, i), cv::Point(-1, -1));
		cv::imshow("Filter", dst);
		cv::waitKey(delay);
	}

	dst = cv::Mat::zeros(image.size(), image.type());
	cv::imshow("Filter", dst);
	cv::waitKey(delay);
	for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 ){
		cv::GaussianBlur(image, dst, cv::Size(i, i), 0, 0);
		cv::imshow("Filter", dst);
		cv::waitKey(delay);
	}

	dst = cv::Mat::zeros(image.size(), image.type());
	cv::imshow("Filter", dst);
	cv::waitKey(delay);
	for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 ){
		cv::medianBlur(image, dst, i);
		cv::imshow("Filter", dst);
		cv::waitKey(delay);
	}

	dst = cv::Mat::zeros(image.size(), image.type());
	cv::imshow("Filter", dst);
	cv::waitKey(delay);
	for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 ){
		cv::bilateralFilter(image, dst, i, 2*i, i/2);
		cv::imshow("Filter", dst);
		cv::waitKey(delay);
	}

	dst = cv::Mat::zeros(image.size(), image.type());
	cv::imshow("Filter", dst);

	cv::waitKey(0);

	return 0;
}
