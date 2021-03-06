#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

cv::Mat image, dst;
char *window_name = "Morphology Window";
int morph_operator;
int morph_elem;
int morph_size;
int max_elem = 2;
int max_operator = 4;
int max_kernel_size = 21;

void Morphology_Operations(int, void*);

int main(int argc, char *argv[]){
	ros::init(argc, argv, "morphology");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	
	image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
	if(!image.data){
		std::cout << "Image error";
		return -1;
	}

	cv::namedWindow(window_name, CV_WINDOW_AUTOSIZE);

	cv::createTrackbar("Operator:\n 0: Opening - 1: Closing \n 2: Gradient - 3: Top Hat \n "
			"4: Black Hat", window_name, &morph_operator, max_operator, Morphology_Operations);

	cv::createTrackbar("Element:\n 0: Rect - 1: Cross - 2: Ellipse", window_name, &morph_elem,
			max_elem, Morphology_Operations);

	cv::createTrackbar("Kernel size:\n 2n +1", window_name, &morph_size,
			max_kernel_size, Morphology_Operations);

	Morphology_Operations(0, 0);

	cv::waitKey(0);
	return 0;
}

void Morphology_Operations(int, void*){
	int operation = morph_operator + 2;
	cv::Mat element = cv::getStructuringElement(morph_elem,
			cv::Size(2*morph_size + 1, 2*morph_size + 1), cv::Point(morph_size, morph_size));
	cv::morphologyEx(image, dst, operation, element);
	cv::imshow(window_name, dst);
}
