#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Point.h>


cv::Mat image;
int thresh_value = 120, max_val = 255;
ros::Publisher point;

void getImage(const sensor_msgs::ImageConstPtr& msg){
	//std::cout << "Captura imagen";
	cv_bridge::CvImagePtr cv_ptr;
	try{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}catch(cv_bridge::Exception & e){
		ROS_ERROR("Exception: %s", e.what());
	}

	image = cv_ptr->image.clone();
	if(!image.data) {
		std::cout << "No imagen";
		cv::waitKey(0);
	}

	cv::namedWindow("imagen", CV_WINDOW_AUTOSIZE);
	cv::imshow("imagen", image);

	cv::Mat gray_scale;
	cv::cvtColor(image, gray_scale, CV_BGR2GRAY);
	cv::namedWindow("gray", CV_WINDOW_AUTOSIZE);

	cv::blur(gray_scale, gray_scale, cv::Size(3, 3));
	cv::imshow("gray", gray_scale);

	cv::Mat threshold_img;
	cv::threshold(gray_scale, threshold_img, thresh_value, max_val, CV_THRESH_BINARY);
	cv::namedWindow("threshold", CV_WINDOW_AUTOSIZE);
	cv::imshow("threshold", threshold_img);
	//cv::waitKey(50);

	cv::Mat canny_img;
	cv::Canny(gray_scale, canny_img, 120, 140);
	cv::namedWindow("canny", CV_WINDOW_AUTOSIZE);
	cv::imshow("canny", canny_img);
	//cv::waitKey(50);

	cv::vector<cv::vector<cv::Point> > contours;
	cv::vector<cv::Vec4i> hierarchy;
	cv::findContours(canny_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE,
			cv::Point(0, 0));

	cv::vector<cv::vector<cv::Point> > hull(contours.size());
	for(int i = 0; i < contours.size(); i++){
		cv::convexHull(cv::Mat(contours[i]), hull[i], false);
	}

	cv::vector<cv::Moments> mu(contours.size());
	for(int i = 0; i < contours.size(); i++){
		mu[i] = moments(contours[i], false );
	}

	cv::vector<cv::Point2f> mc(contours.size() );
	for(int i = 0; i < contours.size(); i++){
		mc[i] = cv::Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);
	}

	float centroX = 0, centroY = 0;
	int num = 0;
	for(int i = 0; i < contours.size(); i++){
		if(!isnanf(mc[i].x) || !isnanf(mc[i].y)){
			centroX += mc[i].x;
			centroY += mc[i].y;
			num++;
		}
	}
	cv::Point2f centro = cv::Point2f(centroX/num, centroY/num);
	//std::cout << "x: " << centro.x << std::endl;
	//std::cout << "y: " << centro.y << std::endl;

	cv::Mat drawing = cv::Mat::zeros(canny_img.size(), CV_8UC3);
	/*for(int i = 0; i < contours.size(); i++){
		cv::drawContours(drawing, contours, i, cv::Scalar(255, 0, 0), 2, 8,
				hierarchy, 0, cv::Point(0, 0));
		cv::drawContours(drawing, hull, i, cv::Scalar(0, 0, 255), 2, 8,
						hierarchy, 0, cv::Point(0, 0));
		cv::circle(drawing, mc[i], 4, cv::Scalar(0, 255, 0), -1, 8, 0);
	}*/
	cv::circle(drawing, centro, 4, cv::Scalar(255, 255, 255), -1, 8, 0);

	cv::namedWindow("contours", CV_WINDOW_AUTOSIZE);
	cv::imshow("contours", drawing);

	geometry_msgs::Point center;
	center.x = 0.79;
	center.y = (centro.x - image.rows/2)/1000;
	center.z = 0.674 - (centro.y - image.rows/2)/1000;

	std::cout << "x: 0.79\n";
	std::cout << "y: " << center.y << std::endl;
	std::cout << "z: " << center.z << std::endl;

	point.publish(center);
	cv::waitKey(500);
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "hand_detection");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	point = nh.advertise<geometry_msgs::Point>("reference_point", 1000);

	image_transport::Subscriber image_sub = it.subscribe("/camera/rgb/image_raw", 1, getImage);

	ros::spin();
	return 0;
}

