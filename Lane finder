#include <opencv2/opencv.hpp>
#include <raspicam_cv.h>
#include <iostream>

using namespace std;
using namespace cv;
using namespace raspicam;

//1280*720 360*240//

Mat frame, Matrix, framePers, frameGray, frameThresh, frameEdge, frameFinal, frameFinalDuplicate; 
Mat ROILane;
int LeftLanePos, RightLanePos;
RaspiCam_Cv Camera; 

vector<int> histrogramLane;

Point2f Source[]={Point2f(70,180),Point2f(260,180),Point2f(10,240),Point2f(315,240)};  //ROI 
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

void Threshold()
{
 cvtColor(framePers,frameGray, COLOR_RGB2GRAY);
 inRange(frameGray, 100, 240, frameThresh);
 Canny(frameGray, frameEdge, 100, 500,3, false);
 add(frameThresh, frameEdge, frameFinal);
 cvtColor(frameFinal, frameFinal, COLOR_GRAY2RGB);
 cvtColor(frameFinal, frameFinalDuplicate, COLOR_RGB2BGR);  //used in histrogram funtion only
}

void Histrogram()
{
  histrogramLane.resize(350);
  histrogramLane.clear();
  
  for(int i=0; i<350; i++)       //frame.size().width =400
  {
      ROILane = frameFinalDuplicate(Rect(i,140,1,100));
      divide(255,ROILane, ROILane);
      histrogramLane.push_back((int)(sum(ROILane)[0]));
      
  }
}

void LaneFinder()
{
    vector<int>:: iterator LeftPtr;
    LeftPtr = max_element(histrogramLane.begin(), histrogramLane.begin() + 150);
    LeftLanePos = distance(histrogramLane.begin(), LeftPtr);
    
    vector<int>:: iterator RightPtr;
    RightPtr = max_element(histrogramLane.begin() +250, histrogramLane.end());
    RightLanePos = distance(histrogramLane.begin(), RightPtr);
    
    line(frameFinal, Point2f(LeftLanePos, 0), Point2f(LeftLanePos, 240), Scalar(0, 255,0), 2);
    line(frameFinal, Point2f(RightLanePos, 0), Point2f(RightLanePos, 240), Scalar(0, 255,0), 2);
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
	    Threshold();
	    Histrogram();
	    LaneFinder();
	    
	    namedWindow("original",WINDOW_KEEPRATIO);
	    moveWindow("original", 0, 100);
	    resizeWindow("original", 448,336);
	    imshow("original", frame);
	    
	    namedWindow("Perspective",WINDOW_KEEPRATIO);
	    moveWindow("Perspective", 448, 100);
	    resizeWindow("Perspective", 448,336);
	    imshow("Perspective", framePers);
	    
	    namedWindow("Final",WINDOW_KEEPRATIO);
	    moveWindow("Final", 896, 100);
	    resizeWindow("Final", 448,336);
	    imshow("Final", frameFinal);
	    
	    
	    waitKey(1);
	    auto end = std::chrono::system_clock::now();
	std::chrono::duration<double> elapsed_seconds = end-start;
    
    float t = elapsed_seconds.count();
    int FPS = 1/t;
    cout<<"FPS = "<<FPS<<endl;
    
	}
		 return 0;
}

