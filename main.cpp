#include<opencv2/videoio.hpp>
#include<opencv2/highgui.hpp>
#include<iostream>
#include<string>
#include<sstream>
#include<tuple>

using namespace cv;
using namespace std;

class Kinect
{
	int dev_id;
	VideoCapture cap;
	double minDepth;
	double maxDepth;
	Mat depth2gray(Mat&);
	void cap_set(VideoCapture&);

public:
	Kinect();
	Kinect(int id, double min, double max);
	~Kinect();
	void grab();
	Mat depth();
	Mat bgr();
};

Kinect::Kinect()
{
	dev_id = 1600;
	minDepth = 800;
	maxDepth = 10000;
	cap.open(dev_id);
	cap_set(cap);
}

Kinect::Kinect(int id, double min, double max)
{
	dev_id = id;
	minDepth = min;
	maxDepth = max;
	cap.open(dev_id);
	cap_set(cap);
}

Kinect::~Kinect()
{
	cout << "Releasing video device" << endl;
	cap.release();
}

Mat Kinect::depth2gray(Mat& input_mat)
{
	Mat depthGray(input_mat.size(), CV_8UC1, Scalar::all(0));
	double depthVal_float;
	unsigned char depthVal_uint8;

	for (auto y = 0; y < input_mat.rows; y++)
	{
		for (auto x = 0; x < input_mat.cols; x++)
		{
			depthVal_float = static_cast<double>(input_mat.at<unsigned short>(y, x));
			if (depthVal_float > minDepth && depthVal_float < maxDepth)
			{
				depthVal_uint8 = static_cast<unsigned char>(floor(((depthVal_float - minDepth) / (maxDepth - minDepth)) * 255.0));
				depthGray.at<unsigned char>(y, x) = depthVal_uint8;
			}
		}
	}
	return depthGray;
}

void Kinect::cap_set(VideoCapture& cap)
{
	cout << "\nKinect between depth range set to: " <<
		minDepth << "mm - " <<
		maxDepth << "mm\n" << endl;

	// map depth pixels to rbg pixels
	cap.set(CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION, 1);
	const auto maxDevDepth = static_cast<double>(cap.get(CAP_PROP_OPENNI_FRAME_MAX_DEPTH));

	if (minDepth > maxDevDepth)
	{
		cerr << "Kinect must be less than " << maxDevDepth << "mm away";
		throw;
	}

	if (maxDepth < 800.0)
	{
		cout << "WARNING! Kinect must be at least 800mm away to work properly\n";
	}

	cout << "\nCAMERA PROPERTIES\n-----------------" << endl;
	cout << "Max Device Depth (mm): " << maxDevDepth << endl;
	cout << "Frame Rate (fps): " << cap.get(CAP_PROP_FPS) << endl;
	cout << "Focal Length (px): " << cap.get(CAP_PROP_OPENNI_FOCAL_LENGTH) << endl;
	cout << "Image Width (px): " << cap.get(CAP_PROP_FRAME_WIDTH) << endl;
	cout << "Image Height (px): " << cap.get(CAP_PROP_FRAME_HEIGHT) << endl;
	cout << "Baseline Depth (mm): " << cap.get(CAP_PROP_OPENNI_BASELINE) << endl;
}

void Kinect::grab()
{
	cap.grab();
}

Mat Kinect::depth()
{
	Mat depth_float;
	Mat depth_img;
	auto depthFail = cap.retrieve(depth_float, CAP_OPENNI_DEPTH_MAP);
	if (!depthFail)
	{
		cout << "Depth frame cap failed" << endl;
	}
	else
	{
		// rescale depth image based on range thresholds and convert to bw
		depth_img = depth2gray(depth_float);
	}
	return depth_img;
}

Mat Kinect::bgr()
{
	Mat bgr_img;
	auto bgrFail = cap.retrieve(bgr_img, CAP_OPENNI_BGR_IMAGE);
	if (!bgrFail)
	{
		cout << "BGR frame cap failed" << endl;
	}
	return bgr_img;
}

tuple<int, double, double> cmd_opts(int argc, char** argv)
{
	int arg_id = CAP_OPENNI2;
	double min_dist = 800.0;
	double max_dist = 10000.0;

	if (argc == 1)
	{
		cout << "using default input arguments" << endl;
	}
	else if (argc == 2)
	{
		stringstream arg0(argv[1]);
		arg0 >> arg_id;
	}
	else if (argc == 3)
	{
		stringstream arg0(argv[1]);
		arg0 >> arg_id;
		stringstream arg1(argv[2]);
		arg1 >> min_dist;
	}
	else if (argc == 4)
	{
		stringstream arg0(argv[1]);
		arg0 >> arg_id;
		stringstream arg1(argv[2]);
		arg1 >> min_dist;
		stringstream arg2(argv[3]);
		arg2 >> max_dist;
	}
	else
	{
		cerr << "too many input arguments" << endl;
		arg_id = - 1;
	}

	return make_tuple(arg_id, min_dist, max_dist);
}

int main(int argc, char** argv)
{
	int input_dev_id;
	double minKinectDistance;
	double maxKinectDistance;

	tie(input_dev_id, minKinectDistance, maxKinectDistance) = cmd_opts(argc, argv);

	if (input_dev_id == -1) return -1;

	Kinect cap(input_dev_id, minKinectDistance, maxKinectDistance);

	namedWindow("DEPTH CAM", WINDOW_AUTOSIZE);
	namedWindow("IMAGE CAM", WINDOW_AUTOSIZE);

	while (true)
	{
		cap.grab();
		auto dep_mat = cap.depth();
		auto bgr_mat = cap.bgr();

		imshow("DEPTH CAM", dep_mat);
		imshow("IMAGE CAM", bgr_mat);

		if (waitKey(1) == 27)
		{
			cout << "\nExiting...\n" << endl;
			destroyAllWindows();
			break;
		}
	}

	return 0;
}
