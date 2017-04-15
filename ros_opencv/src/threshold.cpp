#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

char *window_name = "threshold";
int threshold_type = 3, max_type = 4, max_value = 255, threshold_value = 0;

void threshold_demo(int, void*);

cv::Mat gray_scale, dst;

int main(int argc, char *argv[]){
	ros::init(argc, argv, "threshold");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	
	cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);

	cv::cvtColor(image, gray_scale, CV_BGR2GRAY);

	cv::namedWindow("Original", CV_WINDOW_AUTOSIZE);
	cv::imshow("Original", image);
	cv::namedWindow(window_name, CV_WINDOW_AUTOSIZE);

	cv::createTrackbar("Type: \n 0: Binary \n 1: Binary Inverted \n "
			"2: Truncate \n 3: To Zero \n 4: To Zero Inverted", window_name, &threshold_type,
			max_type, threshold_demo);

	cv::createTrackbar("Value", window_name, &threshold_value, max_value, threshold_demo);

	threshold_demo(0, 0);

	cv::waitKey(0);
	return 0;
}

void threshold_demo(int, void*){
	cv::threshold(gray_scale, dst, threshold_value, max_value, threshold_type);
	cv::imshow(window_name, dst);
}
