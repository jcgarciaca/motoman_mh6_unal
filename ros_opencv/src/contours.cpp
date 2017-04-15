#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


void thresh_callback(int, void*);

int thresh = 100, max_thresh = 255;

cv::Mat gray_scale;

int main(int argc, char *argv[]){
	ros::init(argc, argv, "find_contours");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);

	cv::cvtColor(image, gray_scale, CV_BGR2GRAY);
	cv::blur(gray_scale, gray_scale, cv::Size(3, 3));

	char *window_name = "Source";
	cv::namedWindow(window_name, CV_WINDOW_AUTOSIZE);
	cv::imshow(window_name, image);

	cv::createTrackbar("", window_name, &thresh, max_thresh, thresh_callback);

	thresh_callback(0, 0);
	cv::waitKey(0);
	return 0;
}

void thresh_callback(int, void*){
	cv::Mat canny_output;
	cv::vector<cv::vector<cv::Point> > contours;
	cv::vector<cv::Vec4i> hierarchy;

	cv::Canny(gray_scale, canny_output, thresh, 2*thresh, 3);

	cv::findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE,
			cv::Point(0, 0));

	cv::Mat drawing = cv::Mat::zeros(canny_output.size(), CV_8UC3);
	for(int i = 0; i < contours.size(); i++){
		cv::drawContours(drawing, contours, i, cv::Scalar(255, 255, 255), 2, 8,
				hierarchy, 0, cv::Point(0, 0));
	}
	cv::namedWindow("Contours", CV_WINDOW_AUTOSIZE);
	cv::imshow("Contours", drawing);
}
