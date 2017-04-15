#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


int main(int argc, char *argv[]){
	ros::init(argc, argv, "derivate_image");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	int delta = 0, scale = 1, ddepth = CV_16S;

	cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
	if(!image.data) return -1;
	cv::imshow("Original", image);

	cv::GaussianBlur(image, image, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT);
	cv::imshow("Gaussian Blur", image);

	cv::Mat gray_scale;
	cv::cvtColor(image, gray_scale, CV_BGR2GRAY);

	char *window_name = "edge_detector";
	cv::namedWindow(window_name, CV_WINDOW_AUTOSIZE);

	cv::Mat gradX, gradY, abs_gradX, abs_gradY;
	cv::Sobel(gray_scale, gradX, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
	convertScaleAbs(gradX, abs_gradX);

	cv::Sobel(gray_scale, gradY, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
	convertScaleAbs(gradY, abs_gradY);

	cv::Mat grad;
	cv::addWeighted(abs_gradX, 0.5, abs_gradY, 0.5, 0, grad);

	cv::imshow(window_name, grad);

	cv::waitKey(0);
	return 0;
}
