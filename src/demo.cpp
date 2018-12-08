#include <iostream>
#include <wiringPi.h>
#include <time.h>
#include <stdlib.h>
#include <thread>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <fstream>
#include <vector>
#include <values.h>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#define NSEC_PER_SEC 1000000000

using namespace std;
using namespace boost::asio;
using namespace cv;

struct timespec t;
//the Thread to collect data
void detectData(bool& finish){
	io_service iosev;
	serial_port sp(iosev, "/dev/ttyUSB0");
	sp.set_option(serial_port::baud_rate(115200));
	ofstream outfile;
	outfile.open("test.log");
	char data[10];
	boost::system::error_code err;
	unsigned int i = 0;
	unsigned int iteration = 0;
	unsigned int record_num = 0;
	unsigned int begin_num = 0;
	vector<int> dist_vec(1810);
	outfile << "Odometry 0 0 0" << endl;
	outfile << "Laser 181 ";
	while(iteration <= 15){
		if(record_num >= 181){
			begin_num += 50;
			outfile << endl;
			iteration++;
			if(iteration == 15)
				break;
			record_num = 0;
			double angle = (M_PI * 50/180) * iteration;
		        while(angle > M_PI){
				angle -= M_PI;
			}	
			outfile << "Odometry 0 0 " << angle << endl;
			outfile << "Laser 181 ";
		}
		sp.read_some(buffer(data,9),err);
		if(data[0] == 0x59 && data[1] == 0x59)
		{	
			i++;
			int dist = data[2] + data[3] * 256;
			if(i <= dist_vec.size()){
				dist_vec[i-1] = dist;
			}
			outfile << dist_vec[begin_num + record_num]/100.0 << " ";
			record_num++;
		}
	}
	cout<< "collect data thread end" << endl;
	outfile.close();
	finish = false;
}
// The thread for image recognition
void recognition(bool& finish, double& x, double& y){
	int iLowH = 200/2 ;
	int iHighH = 360/2;

	int iLowS = 5*255/100;
	int iHighS = 30*255/100;
	
	int iLowV=20*255/100;
	int iHighV=50*255/100;
	
	int error_time = 0;
	const int MAX_ERROR = 10;	
	VideoCapture cap(0);
	if(!cap.isOpened()){
		cout << "camera not open" << endl;
		finish = false;
		return;
	}
	Mat originImg;
	while(true){
		bool success = cap.read(originImg);
		if(!success){
			cout << "no image load" << endl;
			finish = false;
			return;
		}
	        Mat imgHSV;
		cvtColor(originImg, imgHSV, COLOR_BGR2HSV);

		Mat imgThresholded;
		inRange(imgHSV,Scalar(iLowH,iLowS,iLowV),Scalar(iHighH,iHighS,iHighV),imgThresholded);
		Mat element = getStructuringElement(MORPH_RECT, Size(5,5));
		morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);		
		morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);
		for(int i = 0; i < 300; i++){
			uchar* data = imgThresholded.ptr<uchar>(i);
			for(int j = 0; j < imgThresholded.cols; j++){
				data[j] = (uchar)1;
			}
		}
		Canny(imgThresholded,imgThresholded,20,80,3,false);

		Mat dst;
		dst = Mat::zeros(imgThresholded.size(), CV_8UC3);
		std::vector<std::vector<Point>>contours;
		std::vector<Vec4i>hierarchy;
		findContours(imgThresholded,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point(0,0));
		vector<Rect> boundRect( contours.size() );
		int num = 0;
		for(int i = 0; i < contours.size(); i++){
			boundRect[i] = boundingRect( Mat(contours[i]) );
			if(boundRect[i].area() > 5000){
				num++;
				rectangle(dst, boundRect[i], Scalar(0,255,0), 1);
				x = boundRect[i].x + boundRect[i].width/2;
				y = boundRect[i].y + boundRect[i].height/2;
			}			
		}
		if(num <= 2 && num >=1){
			if(finish == true){
				cout << "Object Detected, go to approaching" << endl;
				finish = false;
			}
		}else{
			if(finish == false){
				if(error_time >= MAX_ERROR){
					finish = true;
					cout << "Object Miss, go to searching" << endl;
				}else{
					error_time++;
				}
			}
		}
	}
}

		

//main loop for all functions 
int main(){
	cout << "Parameter Initilization" << endl;
	int pinValue = 0;
	unsigned int interval = 20000000;
	unsigned int clk = 1450000;
        unsigned int search = 1540000;
	wiringPiSetup();
	
	clock_gettime(CLOCK_MONOTONIC, &t);
	t.tv_nsec += interval;
	
	pinMode(22,OUTPUT);
	pinMode(23,OUTPUT);
	bool finish = true;
	cout << "collect data thread begin" << endl;
	thread t1(detectData,ref(finish));
	t1.detach();
	while(finish){
		digitalWrite(22, pinValue);
		digitalWrite(23, pinValue);
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
		pinValue = pinValue ^ 1;
		if(pinValue == 1){
			t.tv_nsec += clk;
		}else{
			t.tv_nsec += interval;
		}
		
		while ( t.tv_nsec >= NSEC_PER_SEC) {
			t.tv_nsec -= NSEC_PER_SEC;
			t.tv_sec++;
		}
	}
	digitalWrite(22, 0);
	digitalWrite(23, 0);
	cout << "map construct thread begin" << endl;
	system("rm *.png");
	system("./slam -p test.log");
	cout << "map construct thread end" << endl;

	cout << "image recognition thread begin" << endl;
	pinValue = 0;
        finish = true;
	clock_gettime(CLOCK_MONOTONIC, &t);
	double x =0;
	double y = 0;
	thread th2(recognition, ref(finish), ref(x), ref(y));
	th2.detach();
	t.tv_nsec += interval;
        unsigned int count = 0;
	while(true){
		if(finish){
			if(count > 2000000){
				count = 0;
			}
			if(count > 1000000 && count < 2000000){
				digitalWrite(22,0);
				digitalWrite(23,0);
			}else{
				digitalWrite(22, pinValue);
				digitalWrite(23, pinValue);
				clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
				pinValue = pinValue ^ 1;
				if(pinValue == 1){
					t.tv_nsec += search;
				}else{
					t.tv_nsec += interval;
				}
			}
			count++;		
		
		}else{
			digitalWrite(22,0);
			digitalWrite(23,0);
		}
		while ( t.tv_nsec >= NSEC_PER_SEC) {
			t.tv_nsec -= NSEC_PER_SEC;
			t.tv_sec++;
		}			
	}		
	return 0;
}
