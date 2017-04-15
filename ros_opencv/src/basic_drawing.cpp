#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


void drawLine(cv::Mat img, cv::Point start, cv::Point end){
	int thickness = 2, lineType = 8;
	cv::line(img, start, end, cv::Scalar(255, 255, 255), thickness, lineType);
}

void drawCircle(cv::Mat img, cv::Point center, int radius){
	int thickness = -1, lineType = 8;
	std::cout << "x: " << center.x << std::endl;
	std::cout << "y: " << center.y << std::endl;
	cv::circle(img, center, radius, cv::Scalar(255, 0, 0), thickness, lineType);
}

void drawEllipse(cv::Mat img, double angle, cv::Scalar color){
	int thickness = 1, lineType = 8;
	cv::Point center = cv::Point(img.cols/4, img.rows/2);
	cv::Size axes = cv::Size(img.cols/8, img.rows/8);
	cv::ellipse(img, center, axes, angle, 0, 360, color, thickness, lineType);
}


int main(int argc, char *argv[]){
	ros::init(argc, argv, "basic_drawing");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	
	cv::Mat atom_image = cv::Mat::zeros(400, 800, CV_8UC3);
	drawLine(atom_image, cv::Point(0, 0), cv::Point(300, 100));
	drawCircle(atom_image, cv::Point(atom_image.cols/2, atom_image.rows/2), 50);
	drawEllipse(atom_image, 90, cv::Scalar(255, 255, 255));
	drawEllipse(atom_image, 0, cv::Scalar(255, 0, 0));
	drawEllipse(atom_image, 45, cv::Scalar(0, 255, 0));
	drawEllipse(atom_image, -45, cv::Scalar(0, 0, 255));

	cv::namedWindow("atom_image", CV_WINDOW_AUTOSIZE);
	cv::imshow("atom_image", atom_image);
	cv::waitKey(0);

	return 0;
}
