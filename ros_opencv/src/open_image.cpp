#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


int main(int argc, char *argv[]){
	ros::init(argc, argv, "image_loader");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	cv::Mat imagen = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);

	if(!imagen.data){
		std::cout << "Could not open the image\n";
		return -1;
	}

	cv::namedWindow("Example1", CV_WINDOW_AUTOSIZE);
	cv::imshow("Example1", imagen);
	cv::waitKey(0);
	return 0;
}
