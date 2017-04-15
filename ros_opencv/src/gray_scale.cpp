#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


int main(int argc, char *argv[]){

	ros::init(argc, argv, "Image_changer");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	char *imageName = argv[1];

	cv::Mat image;
	image = cv::imread(imageName, 1);

	if(argc != 2 || !image.data){
		printf("No image data!! \n");
		return -1;
	}

	cv::Mat gray_image;
	cv::cvtColor(image, gray_image, CV_BGR2GRAY);

	cv::namedWindow(imageName, CV_WINDOW_AUTOSIZE);
	cv::namedWindow("Gray image", CV_WINDOW_AUTOSIZE);

	cv::imshow(imageName, image);
	cv::imshow("Gray image", gray_image);

	cv::waitKey(0);

	return 0;
}
