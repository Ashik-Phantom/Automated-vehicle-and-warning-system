#include <opencv2/opencv.hpp>
#include <raspicam_cv.h>
#include <iostream>

using namespace std;
using namespace cv;
using namespace raspicam;

//1280*720 360*240//

Mat frame, Matrix, framePers; 
RaspiCam_Cv Camera; 

Point2f Source[]={Point2f(65,180),Point2f(260,180),Point2f(10,240),Point2f(315,240)};  //ROI 
Point2f Destination[]={Point2f(80,0),Point2f(280,0),Point2f(80,240),Point2f(280,240)};

void Setup ( int argc,char **argv, RaspiCam_Cv &Camera )
  {
    Camera.set ( CAP_PROP_FRAME_WIDTH,  ( "-w",argc,argv,360 ) );
    Camera.set ( CAP_PROP_FRAME_HEIGHT,  ( "-h",argc,argv,240) );
    Camera.set ( CAP_PROP_BRIGHTNESS, ( "-br",argc,argv,50 ) );
    Camera.set ( CAP_PROP_CONTRAST ,( "-co",argc,argv,50 ) );
    Camera.set ( CAP_PROP_SATURATION,  ( "-sa",argc,argv,50 ) );
    Camera.set ( CAP_PROP_GAIN,  ( "-g",argc,argv ,50 ) );
    Camera.set ( CAP_PROP_FPS,  ( "-fps",argc,argv,0));

}

//ROF
void Perspective()
{
	line(frame,Source[0],Source[1], Scalar(0,0,255),2);
	line(frame,Source[1],Source[3], Scalar(0,0,255),2);
	line(frame,Source[3],Source[2], Scalar(0,0,255),2);
	line(frame,Source[2],Source[0], Scalar(0,0,255),2);
	
	
	Matrix = getPerspectiveTransform(Source,Destination);
	warpPerspective(frame,framePers, Matrix, Size(350,240));
}

void Capture()
{
   Camera.grab();
   Camera.retrieve(frame); 
   cvtColor(frame, frame, COLOR_BGR2RGB);
}

int main(int argc,char **argv)
{
	
	Setup(argc, argv, Camera);
	cout<<"connecting to Camera"<<endl;
	if (!Camera.open())
	{
		cout<<"Failed to Connect Camera"<<endl;
		return -1;
	}
		cout<<"Camera ID= "<<Camera.getId()<<endl;
	
	
	
	
	
	while(1)
	{	
	    auto start = std::chrono::system_clock::now();
	    
	    Capture();
	    Perspective();
	    
	    namedWindow("original",WINDOW_KEEPRATIO);
	    moveWindow("original", 50, 100);
	    resizeWindow("original", 640,480);
	    imshow("original", frame);
	    
	    namedWindow("Perspective",WINDOW_KEEPRATIO);
	    moveWindow("Perspective", 700, 100);
	    resizeWindow("Perspective", 640,480);
	    imshow("Perspective", framePers);
	    
	    
	    waitKey(1);
	    auto end = std::chrono::system_clock::now();
	std::chrono::duration<double> elapsed_seconds = end-start;
    
    float t = elapsed_seconds.count();
    int FPS = 1/t;
    cout<<"FPS = "<<FPS<<endl;
    
	}
		 return 0;
}

