#include <opencv2/opencv.hpp>
#include <raspicam_cv.h>
#include <iostream>
#include <wiringPi.h>
#include <chrono>
#include <ctime>

using namespace std;
using namespace cv;
using namespace raspicam;

//1280*720 360*240//
//image processing variable
Mat frame, Matrix, framePers, frameGray, frameThresh, frameEdge, frameFinal, frameFinalDuplicate, frameFinalDuplicate1; 
Mat ROILane, ROILaneEnd;
int LeftLanePos, RightLanePos ,frameCenter,laneCenter,Result, laneEnd;

RaspiCam_Cv Camera; 
stringstream ss;

vector<int> histrogramLane;
vector<int> histrogramLaneEnd;

Point2f Source[]={Point2f(60,180),Point2f(340,180),Point2f(45,212),Point2f(355,212)};  //ROI 
Point2f Destination[]={Point2f(120,0),Point2f(260,0),Point2f(120,240),Point2f(260,240)};

//Machine Learning variables
CascadeClassifier Stop_Cascade;
Mat frame_Stop,RoI_Stop,gray_Stop;
vector<Rect> Stop;
int dist_Stop;

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
   cvtColor(frame, frame_Stop, COLOR_BGR2RGB);
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
 cvtColor(frameFinal, frameFinalDuplicate1, COLOR_RGB2BGR);  //used in histrogram funtion only
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

  histrogramLaneEnd.resize(400);
  histrogramLaneEnd.clear();
  
  for(int i=0; i<400; i++)       //frame.size().width =400
  {
      ROILaneEnd = frameFinalDuplicate1(Rect(i,0,1,240));
      divide(255,ROILaneEnd, ROILaneEnd);
      histrogramLaneEnd.push_back((int)(sum(ROILaneEnd)[0]));
   }
   
   laneEnd = sum(histrogramLaneEnd)[0];
   cout<<"Lane End = "<<laneEnd<<endl;
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


void Stop_detection()
{
    if(!Stop_Cascade.load("//home//pi//Desktop//MACHINE LEARNING//Stop_cascade.xml"))
    {
	printf("Unable to open stop cascade file");
    }
    RoI_Stop = frame_Stop(Rect(200,0,200,140));
    cvtColor(RoI_Stop, gray_Stop, COLOR_RGB2GRAY);
    equalizeHist(gray_Stop, gray_Stop);
    Stop_Cascade.detectMultiScale(gray_Stop, Stop);
    
    for(int i=0; i<Stop.size(); i++)
    {
	Point P1(Stop[i].x, Stop[i].y);
	Point P2(Stop[i].x + Stop[i].width, Stop[i].y + Stop[i].height);
	
	rectangle(RoI_Stop, P1, P2, Scalar(0, 0, 255), 2);
	putText(RoI_Stop, "Stop Sign", P1, FONT_HERSHEY_PLAIN, 1,  Scalar(0, 0, 255, 255), 2);
	dist_Stop = (-1.07)*(P2.x-P1.x) + 102.597;
	
       ss.str(" ");
       ss.clear();
       ss<<"D = "<<dist_Stop<<"cm";
       putText(RoI_Stop, ss.str(), Point2f(1,130), 0,1, Scalar(0,0,255), 2);
    }
    
    
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
	    Stop_detection();
	    
    if (dist_Stop > 5 && dist_Stop < 20)
    {
	digitalWrite(21, 0);
	digitalWrite(22, 0);    //decimal = 8
	digitalWrite(23, 0);
	digitalWrite(24, 1);
	cout<<"Stop Sign"<<endl;
	dist_Stop = 0;
	
	goto Stop_Sign;
    }
    	    
    if (laneEnd > 4000)
    {
	digitalWrite(21, 1);
	digitalWrite(22, 1);    //decimal = 7
	digitalWrite(23, 1);
	digitalWrite(24, 0);
	cout<<"LeneEnd"<<endl;
	delay(3000);
    }
    
    if (Result == 0)
    {
	digitalWrite(21, 0);
	digitalWrite(22, 0);    //decimal = 0
	digitalWrite(23, 0);
	digitalWrite(24, 0);
	cout<<"Forward"<<endl;
    }
    
        
    else if (Result >0 && Result <10)
    {
	digitalWrite(21, 1);
	digitalWrite(22, 0);    //decimal = 1
	digitalWrite(23, 0);
	digitalWrite(24, 0);
	cout<<"Right1"<<endl;
    }
    
        else if (Result >=10 && Result <20)
    {
	digitalWrite(21, 0);
	digitalWrite(22, 1);    //decimal = 2
	digitalWrite(23, 0);
	digitalWrite(24, 0);
	cout<<"Right2"<<endl;
    }
    
        else if (Result >20)
    {
	digitalWrite(21, 1);
	digitalWrite(22, 1);    //decimal = 3
	digitalWrite(23, 0);
	digitalWrite(24, 0);
	cout<<"Right3"<<endl;
    }
    
        else if (Result <0 && Result >-10)
    {
	digitalWrite(21, 0);
	digitalWrite(22, 0);    //decimal = 4
	digitalWrite(23, 1);
	digitalWrite(24, 0);
	cout<<"Left1"<<endl;
    }
    
        else if (Result <=-10 && Result >-20)
    {
	digitalWrite(21, 1);
	digitalWrite(22, 0);    //decimal = 5
	digitalWrite(23, 1);
	digitalWrite(24, 0);
	cout<<"Left2"<<endl;
    }
    
        else if (Result <-20)
    {
	digitalWrite(21, 0);
	digitalWrite(22, 1);    //decimal = 6
	digitalWrite(23, 1);
	digitalWrite(24, 0);
	cout<<"Left3"<<endl;
    }
    
    Stop_Sign:
    
    if (laneEnd > 3000)
    {
       ss.str(" ");
       ss.clear();
       ss<<"lane_End";
       putText(frame, ss.str(), Point2f(1,50), 0,1, Scalar(255,0,0), 2);
    
     }
 	    
    else if (Result == 0)
    {
       ss.str(" ");
       ss.clear();
       ss<<"Result = "<<Result<<" Move Forward";
       putText(frame, ss.str(), Point2f(1,50), 0,1, Scalar(0,0,255), 2);
    
     }
    
    else if (Result > 0)
    {
       ss.str(" ");
       ss.clear();
       ss<<"Result = "<<Result<<" Move Right";
       putText(frame, ss.str(), Point2f(1,50), 0,1, Scalar(0,0,255), 2);
    
     }
     
     else if (Result < 0)
    {
       ss.str(" ");
       ss.clear();
       ss<<"Result = "<<Result<<" Move Left";
       putText(frame, ss.str(), Point2f(1,50), 0,1, Scalar(0,0,255), 2);
    
     }
	    
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
	    
	    namedWindow("Stop Sign", WINDOW_KEEPRATIO);
	    moveWindow("Stop Sign", 896, 400);
	    resizeWindow("Stop Sign", 440, 250);
	    imshow("Stop Sign", RoI_Stop);
	    
	    waitKey(1);
	    auto end = std::chrono::system_clock::now();
	std::chrono::duration<double> elapsed_seconds = end-start;
    
    float t = elapsed_seconds.count();
    int FPS = 1/t;
    cout<<"FPS = "<<FPS<<endl;
    
	}
		 return 0;
}

