#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


int main(int argc, char* argv[]){
	ros::init(argc, argv, "pyramid");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	
	printf( "\n Zoom In-Out demo \n " );
	printf( "------------------ \n" );
	printf( " * [u] -> Zoom in \n" );
	printf( " * [d] -> Zoom out \n" );
	printf( " * [ESC] -> Close program \n \n" );

	cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
	if(!image.data){
		std::cout << "Image error";
		return -1;
	}

	cv::Mat dst, tmp;
	tmp = image;
	dst = tmp;

	char *window_name = "pyramid window";
	cv::namedWindow(window_name, CV_WINDOW_AUTOSIZE);
	cv::imshow(window_name, dst);

	int c;
	while(true){
		c = cv::waitKey(10);
		if((char)c == 27){
			printf("Quit");
			break;
		}
		if((char)c == 'u'){
			cv::pyrUp(tmp, dst, cv::Size(tmp.cols*2, tmp.rows*2));
			printf("** Zoom In: Image x 2 \n");
		}else if((char)c == 'd'){
			cv::pyrDown(tmp, dst, cv::Size(tmp.cols/2, tmp.rows/2));
			printf("** Zoom In: Image / 2 \n");
		}
		cv::imshow(window_name, dst);
		tmp = dst;
	}
	return 0;
}
