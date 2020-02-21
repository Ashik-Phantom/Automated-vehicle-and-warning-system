#include <opencv2/opencv.hpp>
#include <raspicam_cv.h>
#include <iostream>

using namespace std;
using namespace cv;
using namespace raspicam;

//1280*720 360*240//

Mat frame;  

void Setup ( int argc,char **argv, RaspiCam_Cv &Camera )
  {
    Camera.set ( CAP_PROP_FRAME_WIDTH,  ( "-w",argc,argv,1280 ) );
    Camera.set ( CAP_PROP_FRAME_HEIGHT,  ( "-h",argc,argv,720) );
    Camera.set ( CAP_PROP_BRIGHTNESS, ( "-br",argc,argv,50 ) );
    Camera.set ( CAP_PROP_CONTRAST ,( "-co",argc,argv,50 ) );
    Camera.set ( CAP_PROP_SATURATION,  ( "-sa",argc,argv,50 ) );
    Camera.set ( CAP_PROP_GAIN,  ( "-g",argc,argv ,50 ) );
    Camera.set ( CAP_PROP_FPS,  ( "-fps",argc,argv,0));

}

int main(int argc,char **argv)
{
	RaspiCam_Cv Camera;
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
	    Camera.grab();
	    Camera.retrieve(frame);
        
	    imshow("frame", frame);
	    waitKey(1);
	}
		 return 0;
}
