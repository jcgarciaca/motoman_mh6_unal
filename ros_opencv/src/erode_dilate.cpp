#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


int erosion_elem = 0;
int erosion_size = 0;
int dilation_elem = 0;
int dilation_size = 0;
int const max_elem = 2;
int const max_kernel_size = 21;

cv::Mat image, erosion_dst, dilation_dst;

void Erosion( int, void * );
void Dilation( int, void * );

int main(int argc, char *argv[]){
	ros::init(argc, argv, "erode_dilate");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	
	image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
	if(!image.data) return -1;

	cv::namedWindow("Erosion Demo", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("Dilation Demo", CV_WINDOW_AUTOSIZE);
	cv::moveWindow("Dilation Demo", image.cols, 0);

	cv::createTrackbar("Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Erosion Demo",
			&erosion_elem, max_elem, Erosion);

	cv::createTrackbar( "Kernel size:\n 2n +1", "Erosion Demo", &erosion_size, max_kernel_size,
			Erosion);

	cv::createTrackbar("Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Dilation Demo",
			&dilation_elem, max_elem, Dilation);

	cv::createTrackbar( "Kernel size:\n 2n +1", "Dilation Demo", &dilation_size, max_kernel_size,
			Dilation);

	cv::waitKey(0);
	return 0;
}

void Erosion( int, void * ){
	int erosion_type;
	if(erosion_elem == 0) {
		erosion_type = cv::MORPH_RECT;
	}else if(erosion_elem == 1) {
		erosion_type = cv::MORPH_CROSS;
	}else if(erosion_elem == 2) {
		erosion_type = cv::MORPH_ELLIPSE;
	}

	cv::Mat element = cv::getStructuringElement(erosion_type,
			cv::Size(2*erosion_size + 1, 2*erosion_size+1), cv::Point(erosion_size, erosion_size));
	/// Apply the erosion operation
	cv::erode(image, erosion_dst, element);
	cv::imshow( "Erosion Demo", erosion_dst);
}

void Dilation( int, void * ){
	int dilation_type;
	if(dilation_elem == 0) {
		dilation_type = cv::MORPH_RECT;
	}else if(dilation_elem == 1) {
		dilation_type = cv::MORPH_CROSS;
	}else if(dilation_elem == 2) {
		dilation_type = cv::MORPH_ELLIPSE;
	}

	cv::Mat element = cv::getStructuringElement(dilation_type,
			cv::Size(2*dilation_size + 1, 2*dilation_size+1), cv::Point(dilation_size, dilation_size));
	/// Apply the dilation operation
	cv::dilate(image, dilation_dst, element);
	cv::imshow( "Dilation Demo", dilation_dst);
}
