#include <opencv2/opencv.hpp>
#include <raspicam_cv.h>
#include <iostream>

using namespace std;
using namespace cv;
using namespace raspicam;

//1280*720 360*240//

Mat frame, Matrix, framePers, frameGray, frameThresh, frameEdge, frameFinal, frameFinalDuplicate; 
Mat ROILane;
int LeftLanePos, RightLanePos ,frameCenter,laneCenter,Result;

RaspiCam_Cv Camera; 
stringstream ss;

vector<int> histrogramLane;

Point2f Source[]={Point2f(60,180),Point2f(340,180),Point2f(45,212),Point2f(355,212)};  //ROI 
Point2f Destination[]={Point2f(120,0),Point2f(260,0),Point2f(120,240),Point2f(260,240)};

void Setup ( int argc,char **argv, RaspiCam_Cv &Camera )
  {
    Camera.set ( CAP_PROP_FRAME_WIDTH,  ( "-w",argc,argv,400) );
    Camera.set ( CAP_PROP_FRAME_HEIGHT,  ( "-h",argc,argv,240) );
    Camera.set ( CAP_PROP_BRIGHTNESS, ( "-br",argc,argv,50 ) );
    Camera.set ( CAP_PROP_CONTRAST ,( "-co",argc,argv,50 ) );
    Camera.set ( CAP_PROP_SATURATION,  ( "-sa",argc,argv,50 ) );
    Camera.set ( CAP_PROP_GAIN,  ( "-g",argc,argv ,50 ) );
    Camera.set ( CAP_PROP_FPS,  ( "-fps",argc,argv,0));

}

void Capture()
{
   Camera.grab();
   Camera.retrieve(frame); 
   cvtColor(frame, frame, COLOR_BGR2RGB);
   
}

//ROF
void Perspective()
{
	line(frame,Source[0],Source[1], Scalar(0,0,255),2);
	line(frame,Source[1],Source[3], Scalar(0,0,255),2);
	line(frame,Source[3],Source[2], Scalar(0,0,255),2);
	line(frame,Source[2],Source[0], Scalar(0,0,255),2);
	
	
	Matrix = getPerspectiveTransform(Source,Destination);
	warpPerspective(frame,framePers, Matrix, Size(400,240));
}

void Threshold()
{
 cvtColor(framePers,frameGray, COLOR_RGB2GRAY);
 inRange(frameGray, 110, 255, frameThresh);
 Canny(frameGray, frameEdge, 200, 400,3, false);
 add(frameThresh, frameEdge, frameFinal);
 cvtColor(frameFinal, frameFinal, COLOR_GRAY2RGB);
 cvtColor(frameFinal, frameFinalDuplicate, COLOR_RGB2BGR);  //used in histrogram funtion only
 	
}

void Histrogram()
{
  histrogramLane.resize(400);
  histrogramLane.clear();
  
  for(int i=0; i<400; i++)       //frame.size().width =400
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

void LaneCenter()
{
    laneCenter=(RightLanePos-LeftLanePos)/2+LeftLanePos;
    frameCenter = 190;
    
    line(frameFinal, Point2f(laneCenter,0),Point2f(laneCenter,240), Scalar(0,255,0),3);
    line(frameFinal, Point2f(frameCenter,0),Point2f(frameCenter,240), Scalar(255,0,0),3);
    
    Result = laneCenter-frameCenter;
}


int main(int argc,char **argv)
{
	wiringPiSetup();
	pinMode(21, OUTPUT);
	pinMode(22, OUTPUT);
	pinMode(23, OUTPUT);
	pinMode(24, OUTPUT);
	
	Setup(argc, argv, Camera);
	cout<<"connecting to Camera"<<endl;
	if (!Camera.open())
	{
		cout<<"Failed to Connect Camera"<<endl;
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
	    LaneCenter();
 
	    namedWindow("original",WINDOW_KEEPRATIO);
	    moveWindow("original", 0, 100);
	    resizeWindow("original", 440,250);
	    imshow("original", frame);
	    
	    namedWindow("Perspective",WINDOW_KEEPRATIO);
	    moveWindow("Perspective", 440, 100);
	    resizeWindow("Perspective", 448,250);
	    imshow("Perspective", framePers);
	    
	    namedWindow("Final",WINDOW_KEEPRATIO);
	    moveWindow("Final", 896, 100);
	    resizeWindow("Final", 440,250);
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

