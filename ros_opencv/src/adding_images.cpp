#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


int main(int argc, char *argv[]){

	double alpha = 0.5, beta;

	ros::init(argc, argv, "adding_images");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	char *imageName1 = argv[1];
	char *imageName2 = argv[2];
	
	cv::Mat image1 = cv::imread(imageName1, CV_LOAD_IMAGE_COLOR);
	cv::Mat image2 = cv::imread(imageName2, CV_LOAD_IMAGE_COLOR);
	cv::Mat dst;

	if(!image1.data){
		printf("error imagen 1\n");
		return -1;
	}

	if(!image2.data){
		printf("error imagen 2\n");
		return -1;
	}

	cv::namedWindow(imageName1, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(imageName2, CV_WINDOW_AUTOSIZE);
	cv::namedWindow("Blended image", CV_WINDOW_AUTOSIZE);

	beta = 1 - alpha;

	cv::addWeighted(image1, alpha, image2, beta, 0.0, dst);

	cv::imshow(imageName1, image1);
	cv::imshow(imageName2, image2);
	cv::imshow("Blended image", dst);

	cv::waitKey(0);

	return 0;
}
