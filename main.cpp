#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#include <sstream>

using namespace cv;
using namespace std;

void depth2gray(const Mat& depthImg, Mat& bwImg, double& depthMin, double& depthMax)
{
	double depthVal_float;
	unsigned char depthVal_uint8;

	for (auto y = 0; y < depthImg.rows; y++)
	{
		for (auto x = 0; x < depthImg.cols; x++)
		{
			depthVal_float = static_cast<double>(depthImg.at<unsigned short>(y, x));
			if (depthVal_float > depthMin && depthVal_float < depthMax)
			{
				depthVal_uint8 = static_cast<unsigned char>(floor(((depthVal_float - depthMin) / (depthMax - depthMin)) * 255.0));
				bwImg.at<unsigned char>(y, x) = depthVal_uint8;
			}
		}
	}
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
	else if (argc == 2)
	{
		maxKinectDistance = 2400.0;
		stringstream arg1(argv[1]);
		arg1 >> minKinectDistance;
	}
	else if (argc == 3)
	{
		stringstream arg1(argv[1]);
		stringstream arg2(argv[2]);
		arg1 >> minKinectDistance;
		arg2 >> maxKinectDistance;
	}
	else
	{
		cerr << "too many input arguments" << endl;
	}

	cout << "\nKinect between depth range: " <<
		minKinectDistance << "mm - " <<
		maxKinectDistance << "mm\n" << endl;

	// open the Kinect camera
	VideoCapture cap(CAP_OPENNI2);

	// check if we succeeded
	if (!cap.isOpened())
		return -1;

	// map depth pixels to rbg pixels
	cap.set(CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION, 1);
	const auto maxDepth = static_cast<double>(cap.get(CAP_PROP_OPENNI_FRAME_MAX_DEPTH));

	if (maxKinectDistance > maxDepth)
	{
		cout << "Kinect must be less than " << maxDepth << "mm away";
		return(-1);
	}

	if (minKinectDistance < 800.0)
	{
		cout << "WARNING! Kinect must be at least 800mm away to work properly\n";
	}

	cout << "\nCAMERA PROPERTIES\n-----------------" << endl;
	cout << "Max Depth (mm): " << maxDepth << endl;
	cout << "Frame Rate (fps): " << cap.get(CAP_PROP_FPS) << endl;
	cout << "Focal Length (px): " << cap.get(CAP_PROP_OPENNI_FOCAL_LENGTH) << endl;
	cout << "Image Width (px): " << cap.get(CAP_PROP_FRAME_WIDTH) << endl;
	cout << "Image Height (px): " << cap.get(CAP_PROP_FRAME_HEIGHT) << endl;
	cout << "Baseline Depth (mm): " << cap.get(CAP_PROP_OPENNI_BASELINE) << endl;

	namedWindow("DEPTH CAM", WINDOW_AUTOSIZE);
	namedWindow("IMAGE CAM", WINDOW_AUTOSIZE);

	while (true)
	{
		// grab and decode frames
		Mat depthFrame;
		Mat bgrFrame;

		cap.grab();

		auto depthFail = cap.retrieve(depthFrame, CAP_OPENNI_DEPTH_MAP);
		auto bgrFail = cap.retrieve(bgrFrame, CAP_OPENNI_BGR_IMAGE);

		if (!bgrFail || !depthFail)
		{
			cout << "Frame cap failed" << endl;
		}
		else
		{
			// rescale depth image based on range thresholds and convert to bw
			Mat depthGray(depthFrame.size(), CV_8UC1, Scalar::all(0));
			depth2gray(depthFrame, depthGray, minKinectDistance, maxKinectDistance);
			imshow("DEPTH CAM", depthGray);
			imshow("IMAGE CAM", bgrFrame);
		}

		if (waitKey(1) == 27)
		{
			cout << "\nExiting...\n" << endl;
			destroyAllWindows();
			break;
		}
	}

	return 0;
}
