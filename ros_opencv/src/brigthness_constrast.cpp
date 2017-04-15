#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


int main(int argc, char *argv[]){

	double alpha, beta;

	ros::init(argc, argv, "brigthness_constrast");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	
	cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
	cv::Mat new_image = cv::Mat::zeros(image.size(), image.type());
	std::cout << "Tipo: " << image.type() << std::endl;
	std::cout << "Transformacion lineal" << std::endl;
	std::cout << "Ingrese el valor alpha [1-3]: ";
	std::cin >> alpha;
	std::cout << "Ingrese el valor de beta [0 - 100]: ";
	std::cin >> beta;
	std::cout << std::endl;

	for(int i = 0; i < image.cols; i++){
		for(int j = 0; j < image.rows; j++){
			for(int c = 0; c < 3; c++){
				new_image.at<cv::Vec3b>(j, i)[c] =
						cv::saturate_cast<uchar>( alpha * image.at<cv::Vec3b>(j, i)[c] + beta);
			}			
		}
	}

	cv::namedWindow("Original image", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("New image", CV_WINDOW_AUTOSIZE);

	cv::imshow("Original image", image);
	cv::imshow("New image", new_image);

	cv::waitKey(0);

	return 0;
}
