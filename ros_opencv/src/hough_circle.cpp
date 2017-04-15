#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


int main(int argc, char *argv[]){
	ros::init(argc, argv, "hough_circle");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
	if(!image.data) return -1;
	cv::Mat gray_scale;
	cv::cvtColor(image, gray_scale, CV_BGR2GRAY);

	cv::GaussianBlur(gray_scale, gray_scale, cv::Size(9, 9), 2, 2);
	cv::vector<cv::Vec3f> circles;

	cv::HoughCircles(gray_scale, circles, CV_HOUGH_GRADIENT, 1, gray_scale.rows/8, 200, 100, 0, 0);

	std::cout << "Circulos: " << circles.size() << "\n";

	for(int i = 0; i < circles.size(); i++){
		cv::Point center(circles[i][0], circles[i][1]);
		int radius = circles[i][2];
		cv::circle(image, center, 3, cv::Scalar(0, 255, 0), -1, 8);
		cv::circle(image, center, radius, cv::Scalar(0, 0, 255), 2, 8);
	}

	cv::namedWindow("Hough Circle", CV_WINDOW_AUTOSIZE);
	cv::imshow("Hough Circle", image);
	cv::waitKey(0);
	return 0;
}
