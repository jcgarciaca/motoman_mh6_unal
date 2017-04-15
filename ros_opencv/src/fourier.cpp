#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


int main(int argc, char *argv[]){
	ros::init(argc, argv, "fourier");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	
	char *imageName = argv[1];
	cv::Mat image = cv::imread(imageName, CV_LOAD_IMAGE_GRAYSCALE);

	if(!image.data){
		std::cout << "Error en la imagen\n";
		return -1;
	}

	int m = cv::getOptimalDFTSize(image.rows);
	int n = cv::getOptimalDFTSize(image.cols);

	cv::Mat padded;
	cv::copyMakeBorder(image, padded, 0, m - image.rows, 0, n - image.cols,
			cv::BORDER_CONSTANT, cv::Scalar::all(0));
	cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
	cv::Mat complex;
	cv::merge(planes, 2, complex);
	cv::dft(complex, complex);
	cv::split(complex, planes);
	cv::magnitude(planes[0], planes[1], planes[0]);
	cv::Mat mag = planes[0];
	mag += cv::Scalar::all(1);
	cv::log(mag, mag);
	mag = mag(cv::Rect(0, 0, mag.cols & -2, mag.rows & -2));

	int cx = mag.cols/2;
	int cy = mag.rows/2;

	cv::Mat q0(mag, cv::Rect(0, 0, cx, cy));
	cv::Mat q1(mag, cv::Rect(cx, 0, cx, cy));
	cv::Mat q2(mag, cv::Rect(0, cy, cx, cy));
	cv::Mat q3(mag, cv::Rect(cx, cy, cx, cy));

	cv::Mat tmp;
	q0.copyTo(tmp);
	q3.copyTo(q0);
	tmp.copyTo(q3);

	q1.copyTo(tmp);
	q2.copyTo(q1);
	tmp.copyTo(q2);

	cv::normalize(mag, mag, 0, 1, CV_MINMAX);


	//cv::namedWindow(imageName, CV_WINDOW_AUTOSIZE);
	cv::imshow("Input image", image);
	cv::imshow("Spectrum magnitude", mag);
	cv::waitKey(0);

	return 0;
}
