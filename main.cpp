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
private:
	int dev_id;
	VideoCapture cap;
	double minDepth;
	double maxDepth;
	double rngDepth;
	void kinect_init(int&);
	Mat depth2gray(Mat&);
	Mat depth2col(Mat&);
	uchar col_rescale(double v, double m);
public:
	Kinect();
	Kinect(int id, double min, double max);
	~Kinect();
	void grab();
	Mat depth(bool&);
	Mat bgr();
};

Kinect::Kinect()
{
	dev_id = CAP_OPENNI2;
	minDepth = 475;
	maxDepth = 10000;
	kinect_init(dev_id);
}

Kinect::Kinect(int id, double min, double max)
{
	dev_id = id;
	minDepth = min;
	maxDepth = max;
	kinect_init(dev_id);
}

Kinect::~Kinect()
{
	cout << "Releasing video device" << endl;
	cap.release();
}

void Kinect::kinect_init(int& id)
{
	cap.open(id);

	// map depth pixels to rbg pixels
	cap.set(CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION, 1);

	// device depth info
	const auto minDevDepth = static_cast<double>(cap.get(CAP_PROP_OPENNI_BASELINE));
	const auto maxDevDepth = static_cast<double>(cap.get(CAP_PROP_OPENNI_FRAME_MAX_DEPTH));

	minDepth = minDepth + minDevDepth;
	rngDepth = maxDepth - minDepth;

	cout << "\n\nCAMERA PROPERTIES\n"
		"--------------------------------------------------" << endl;
	cout << "Baseline value added to min distance: (mm) "
		<< minDevDepth << endl;
	cout << "Maximum device depth distance:        (mm) "
		<< maxDevDepth << endl;
	cout << "Focal length:                         (px) "
		<< cap.get(CAP_PROP_OPENNI_FOCAL_LENGTH) << endl;
	cout << "Image width:                          (px) "
		<< cap.get(CAP_PROP_FRAME_WIDTH) << endl;
	cout << "Image height:                         (px) "
		<< cap.get(CAP_PROP_FRAME_HEIGHT) << endl;
	cout << "Frame rate:                          (fps) "
		<< cap.get(CAP_PROP_FPS) << endl;
	cout << "\nKinect depth quantized based on range: "
		<< minDepth << "mm - " << maxDepth << "mm\n" << endl;
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

Mat Kinect::depth2col(Mat& input_mat)
{
	Mat depthCol(input_mat.size(), CV_8UC3, Scalar::all(255));
	double depth_normalized;
	uchar b_uint8;
	uchar g_uint8;
	uchar r_uint8;

	for (auto m = 0; m < input_mat.rows; m++)
	{
		for (auto n = 0; n < input_mat.cols; n++)
		{
			depth_normalized = (static_cast<double>(input_mat.at<unsigned short>(m, n)) - minDepth) / rngDepth;

			if (depth_normalized > 0 && depth_normalized <= 1)
			{ // white = out of range

				if (depth_normalized > 0 && depth_normalized <= 0.167)
				{ // red = [0 0 1-255]
					b_uint8 = 0;
					g_uint8 = 0;
					r_uint8 = col_rescale(depth_normalized, 0);
				}
				else if (depth_normalized > 0.167 && depth_normalized <= 0.333)
				{ // yellow = [0 1-255 1-255]
					b_uint8 = 0;
					g_uint8 = col_rescale(depth_normalized, 0.167);
					r_uint8 = g_uint8;
				}
				else if (depth_normalized > 0.333 && depth_normalized <= 0.5)
				{ // green = [0 1-255 0]
					b_uint8 = 0;
					g_uint8 = col_rescale(depth_normalized, 0.333);
					r_uint8 = 0;
				}
				else if (depth_normalized > 0.5 && depth_normalized <= 0.67)
				{ // cyan = [1-255 1-255 0]
					b_uint8 = col_rescale(depth_normalized, 0.5);
					g_uint8 = b_uint8;
					r_uint8 = 0;
				}
				else if (depth_normalized > 0.67 && depth_normalized <= 0.833)
				{ // blue = [1-255 0 0]
					b_uint8 = col_rescale(depth_normalized, 0.67);
					g_uint8 = 0;
					r_uint8 = 0;
				}
				else if (depth_normalized > 0.833 && depth_normalized <= 1)
				{ // mag = [1-255 0 1-255]
					b_uint8 = col_rescale(depth_normalized, 0.833);
					g_uint8 = 0;
					r_uint8 = b_uint8;
				}
				else
				{
					b_uint8 = 0;
					g_uint8 = 0;
					r_uint8 = 0;
					cout << "error finding color for depth value" << endl;
				}
				depthCol.at<Vec3b>(m, n)[0] = b_uint8;
				depthCol.at<Vec3b>(m, n)[1] = g_uint8;
				depthCol.at<Vec3b>(m, n)[2] = r_uint8;
			}
		}
	}
	return depthCol;
}

uchar Kinect::col_rescale(double v, double m)
{
	double val = ceil(((v - m) / 0.167) * 255);
	return static_cast<uchar>(val);
}

void Kinect::grab()
{
	cap.grab();
}

Mat Kinect::depth(bool& quantize)
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
		if (quantize)
		{
			// rescale depth image based on range thresholds and convert to bw
			depth_img = depth2gray(depth_float);
		}
		else
		{
			depth_img = depth2col(depth_float);
		}
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

tuple<int, double, double, bool> cmd_opts(int argc, char** argv)
{
	int arg_id = CAP_OPENNI2;
	double min_dist = 475;
	double max_dist = 10000;
	bool quantize = false;

	if (argc == 1)
	{
		cout << "\nUsing default input arguments" << endl;
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
	else if (argc == 5)
	{
		stringstream arg0(argv[1]);
		arg0 >> arg_id;
		stringstream arg1(argv[2]);
		arg1 >> min_dist;
		stringstream arg2(argv[3]);
		arg2 >> max_dist;
		stringstream arg3(argv[4]);
		arg3 >> quantize;
	}
	else
	{
		cerr << "\nToo many input arguments" << endl;
		arg_id = -100;
	}

	if (min_dist < 475)
	{
		cout << "\nWARNING: some close range values will not be used!" << endl;
	}

	if (min_dist >= max_dist)
	{
		cerr << "\nMax distance must be greater than min distance" << endl;
		arg_id = -100;
	}

	if (max_dist < 475)
	{
		cerr << "\nMax distance cannot be less than 475 for Kinect to work properly" << endl;
		arg_id = -100;
	}

	return make_tuple(arg_id, min_dist, max_dist, quantize);
}

int main(int argc, char** argv)
{
	int input_dev_id;
	double minKinectDistance;
	double maxKinectDistance;
	bool more_quantize;

	tie(input_dev_id, minKinectDistance, maxKinectDistance, more_quantize) = cmd_opts(argc, argv);

	if (input_dev_id == -100) return -1;

	Kinect cap(input_dev_id, minKinectDistance, maxKinectDistance);

	namedWindow("DEPTH CAM", WINDOW_AUTOSIZE);
	namedWindow("IMAGE CAM", WINDOW_AUTOSIZE);

	while (true)
	{
		cap.grab();
		auto dep_mat = cap.depth(more_quantize);
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
