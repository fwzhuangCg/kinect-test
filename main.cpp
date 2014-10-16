#include "opencv2/videoio/videoio.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace cv;
using namespace std;

static void depth2gray( const Mat& depthImg, Mat& bwImg, double& depthMin, double& depthMax)
{
    double depthVal_float = 0.0;
    unsigned char depthVal_uint8 = 0;

    for( int y = 0; y < depthImg.rows; y++ )
    {
        for( int x = 0; x < depthImg.cols; x++ )
        {
            depthVal_float = (double)depthImg.at<unsigned short>(y,x);
            if (depthVal_float > depthMin && depthVal_float < depthMax)
            {
                depthVal_uint8 = (unsigned char)floor(((depthVal_float-depthMin) / (depthMax-depthMin)) * 255.0);
                bwImg.at<unsigned char>(y,x) = depthVal_uint8;
            }
        }
    }
}

void timeDuration(double& dur, double& tickOn, double& tickOff)
{
    dur = 1000 * ((tickOff - tickOn) / getTickFrequency());
}

void printFPS(double& tickOn, double& tickOff)
{
    cout << getTickFrequency() / (tickOff - tickOn) << endl;
}

bool pauseCap(double& durMs, double& frameDur)
{

    if (durMs < frameDur)
    {
        if (waitKey(frameDur - durMs) >= 0) return true;
    }
    else
    {
        if (waitKey(1) >= 0) return true;
    }

    return false;
}

int main(int argc, char* argv[])
{

    double minKinectDistance;
    double maxKinectDistance;

    if (argc == 1)
    {
        minKinectDistance = 800.0;
        maxKinectDistance = 1750.0;
    }
    else
    {
        minKinectDistance = atof(argv[1]);
        maxKinectDistance = atof(argv[2]);
    }

    cout << "\nTry to position the Kinect between " <<
            minKinectDistance << " & " <<
            maxKinectDistance << " mm\n" << endl;


    // open the Kinect camera
    VideoCapture cap(CAP_OPENNI2);

    // check if we succeeded
    if(!cap.isOpened())
        return -1;

    // map depth pixels to rbg pixels
    cap.set(CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION, 1);
    const double maxDepth = (double)cap.get(CAP_PROP_OPENNI_FRAME_MAX_DEPTH);

    if (maxKinectDistance > maxDepth)
    {
        cout << "Kinect must be less than " << maxDepth << "mm away";
        return(-1);
    }

    if (minKinectDistance < 800.0)
    {
        cout << "Kinect must be at least 80 cm away to work properly";
        return(-1);
    }

    cout << "\nCAMERA PROPERTIES\n-----------------" << endl;
    cout << "Max Depth (mm): " << maxDepth << endl;
    cout << "Frame Rate (fps): " << cap.get(CAP_PROP_FPS) << endl;
    cout << "Focal Length (px): " << cap.get(CAP_PROP_OPENNI_FOCAL_LENGTH) << endl;
    cout << "Image Width (px): " << cap.get(CAP_PROP_FRAME_WIDTH) << endl;
    cout << "Image Height (px): " << cap.get(CAP_PROP_FRAME_HEIGHT) << endl;
    cout << "Baseline Depth (mm): " << cap.get(CAP_PROP_OPENNI_BASELINE) << endl;

    namedWindow("DEPTH CAM", 1);
    namedWindow("IMAGE CAM", 1);

    double tickStart = (double)getTickCount();
    double tickEnd = tickStart;


    double frames_per_sec = 20.0;
    double frameDuration_ms = 1000 / frames_per_sec;
    double loopDuration = frameDuration_ms;

    for(;;)
    {

        // calculate loop time from tickStart to tickEnd
        timeDuration(loopDuration, tickStart, tickEnd);

        // decide if to pause if loop is too fast
        // if loop is too slow you must lower frame rate capture
        if (pauseCap(loopDuration, frameDuration_ms)) break;

        // start timer
        tickStart = (double)getTickCount();

        // grab and decode frames
        Mat depthFrame;
        Mat bgrFrame;

        cap.grab();

        bool depthFail = cap.retrieve(depthFrame, CAP_OPENNI_DEPTH_MAP);
        bool bgrFail = cap.retrieve(bgrFrame, CAP_OPENNI_BGR_IMAGE);

        if (!bgrFail || !depthFail)
        {
            cout << "Frame cap failed" << endl;
        }

        else
        {

            // rescale depth image based on range thresholds and convert to bw
            Mat depthGray(depthFrame.size(), CV_8UC1, Scalar::all(0));
            depth2gray(depthFrame, depthGray, minKinectDistance, maxKinectDistance);

            //            unsigned char val = depthGray.at<unsigned char>(240,320);
            //            cout << "Mid distance (mm): " << (int)val << endl;

            imshow("DEPTH CAM", depthGray);
            imshow("IMAGE CAM", bgrFrame);
        }


        // end timer
        tickEnd = (double)getTickCount();



        //        cout << tickStart << endl;
        //        cout << tickEnd << endl;
        //        cout << loopDuration << endl;
        //        printFPS(tickStart, tickEnd);



    }

    return 0;
}
