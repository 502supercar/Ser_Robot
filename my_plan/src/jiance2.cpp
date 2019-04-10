#include <iostream>
#include <sl/Camera.hpp>
#include <sl/types.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <algorithm>
#include <string>
#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#define unsigned char uchar;

using namespace std;
using namespace cv;

const double thre = 0.36; //0.38,0.28
const double surf_thre = 1800.0;//2200
//const int match_disten = 150;//70
const int match_x = 40;
const int match_y = 100;
int i, j, k;
int t = 1;
int rec[4] = { 0 };
cv::Mat frame;
cv::Mat img1, img2, imgROI;
std::vector < cv::KeyPoint > keypoints1, keypoints2;
cv::Mat descriptors1, descriptors2;
vector < cv::DMatch > matches;
vector < Point2f > points2, pot;
Point2f central_point = Point2f(-1.0, -1.0);
Point ptl = Point(0, 0);

Ptr < Feature2D > surf = xfeatures2d::SURF::create(surf_thre, 4, 3, 0, 1);
cv::BFMatcher matcher;


void on_mouse1(int event, int x, int y, int flags, void *ustc);
void make_mark(cv::Mat& mark);
void sharpen(cv::Mat &img, cv::Mat &ret);
cv::RotatedRect createRotatedRect(cv::Rect box);
int obj_find(cv::Mat& frame, cv::Mat mark);
cv::Mat slMat2cvMat(sl::Mat& input);
bool tf_broadcast(tf::TransformBroadcaster &br,tf::Transform &transform,double x,double y,double z);

int main(int argc, char **argv)
{
	ros::init(argc,argv,"jiance2");
  ros::NodeHandle n;

  tf::TransformBroadcaster br;
  tf::Transform transform;

	sl::Camera zed;
	sl::InitParameters init_params;
	init_params.camera_resolution = sl::RESOLUTION_HD720;//HD720
	//init_params.camera_fps = 30;//ʵʱ��Ƶʱ��������֡�ʣ���������
	init_params.depth_mode = sl::DEPTH_MODE_MEDIUM; // ģʽ����-ULTRA��QUALITY��MEDIUM��PERFORMANCE
	init_params.coordinate_units = sl::UNIT_MILLIMETER; // ���������Ϣ�ĵ�λ/UNIT_CENTIMETER,UNIT_MILLIMETER,UNIT_METER
	init_params.depth_minimum_distance = 300; // Set the minimum depth perception distance to 30cm
	init_params.coordinate_system = sl::COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD;
	//zed.setDepthMaxRangeValue(30);//������Ϊ40��
	//sl::RuntimeParameters runtime_parameters;
	//runtime_parameters.sensing_mode = sl::SENSING_MODE_STANDARD; //FILL,STANDARD

	img1 = cv :: imread("/home/lcq/jaco_ws/src/kinova-ros/my_plan/src/2.jpg");
	if (!img1.data)
	{
		cout << "error in reading image!!! " << endl;
		return -3;
	}
	cv::Mat temp1;
	cv::medianBlur(img1, img1, 3);
	sharpen(img1, temp1);
	img1 = temp1.clone();
	cv::imshow("img1", img1);
	surf->detect(img1, keypoints1);
	surf->compute(img1, keypoints1, descriptors1);
	std::cout << "keypoints sum: " << keypoints1.size() << std::endl;
	if (keypoints1.size() < 25) t = 0;

	sl::ERROR_CODE err = zed.open(init_params);
	if (err != sl::SUCCESS)
	{
		cout << "error!!" << endl;
		zed.close();
		return -2;
	}
	sl::Mat zed_img(zed.getResolution(), sl::MAT_TYPE_8U_C4);
	//sl::Mat depth_map(zed.getResolution(), sl::MAT_TYPE_8U_C4);
	sl::Mat cloud_map(zed.getResolution(), sl::MAT_TYPE_32F_C4);
	cv::Mat frame_img = slMat2cvMat (zed_img);
	//cv::Mat frame_depth = slMat2cvMat (depth_map);
	//float depth_value = 0;
	sl :: float4 point3D;
	//sl :: float4 pointbuchang;
	float x = 0.0;
	float y = 0.0;
	float z = 0.0;
	//float xz = zed.getResolution().width / 2;
	//float yz = zed.getResolution().height / 2;
	cv::Mat mark(cv::Size(zed.getResolution().width, zed.getResolution().height), CV_8UC1, cv::Scalar::all(0));
	make_mark(mark);

	char c;
	while ( t || n.ok())
	{
		if (zed.grab() == sl::SUCCESS)
		{
			zed.retrieveImage(zed_img, sl::VIEW_LEFT);
			//zed.retrieveImage(depth_map, sl::VIEW_DEPTH);
			zed.retrieveMeasure(cloud_map, sl::MEASURE_XYZ, sl::MEM_GPU);//zed.getResolution().width, zed.getResolution().height
		}
		else
		{
			cout << "error!!!" << endl;
      continue;
			//zed.close();
			//return -1;
		}
		frame = frame_img.clone();
		cv::medianBlur(frame, frame, 3);
		sharpen(frame, temp1);
		frame = temp1.clone();
		if (ptl.x != 0 && ptl.y != 0) circle(frame, ptl, 2, cv::Scalar(255, 0, 255), CV_FILLED, CV_AA);
		obj_find(frame, mark);
		cv::imshow( "[result]", frame);
		cvSetMouseCallback("[result]", on_mouse1, NULL);
		//cv::imshow( "depth", frame_depth);
		//depth_map.getValue(i, j, &depth_value);

		if (central_point.x >= 0 && central_point.y >= 0)
		{
			cloud_map.getValue(central_point.x, central_point.y, &point3D, sl::MEM_GPU);
			//cloud_map.getValue(xz, yz, &pointbuchang, sl::MEM_GPU);
			x = point3D.x;
			y = point3D.y;
			z = point3D.z;
			cout << "object's coordinate is: ( "<< x << ',' << y << ',' << z <<" )" << endl;
			tf_broadcast(br,transform,x/1000.0,y/1000.0,z/1000.0);
		}
		else
		{
      cout<<"Can't find object"<<endl;
			tf_broadcast(br,transform,0.0,0.0,0.0);
		}

		c = cv::waitKey(10);
		if (c == 27 || c == ' ') break;

	}
	cv::waitKey(0);
	cloud_map.free(sl::MEM_GPU);
	zed.close();

  ros::spin();
	return 0;
}

cv::Mat slMat2cvMat(sl::Mat& input) {
	using namespace sl;
	// Mapping between MAT_TYPE and CV_TYPE 
	int cv_type = -1;
	switch (input.getDataType()) {
	case MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
	case MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
	case MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
	case MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
	case MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
	case MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
	case MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
	case MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
	default: break;
	}
	
	cv::Mat temp(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
	/*vector <cv::Mat> channels;
	cv::split( temp, channels);
	channels.pop_back();
	cv::Mat output(input.getHeight(), input.getWidth(), CV_8UC3);
	cv::merge(channels, output);*/

	return temp;

}

void on_mouse1(int event, int x, int y, int flags, void *ustc)
{
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		ptl = Point(x, y);
		cout << "( " << x << ", " << y << " )" << endl;
	}
}

void make_mark(cv::Mat& mark)
{
	for (i = 0; i <= mark.rows / 2; i++)
		for (j = mark.cols / 5; j < mark.cols / 5 * 4; j++)
		{
			mark.at <uchar>(i, j) = 255;
		}
}

cv::RotatedRect createRotatedRect(cv::Rect box)
{
	cv::Point2f p_temp[3];
	p_temp[0] = cv::Point2f(box.x, box.y);
	p_temp[1] = cv::Point2f(box.x + box.width, box.y);
	p_temp[2] = cv::Point2f(box.x + box.width, box.y + box.height);
	return (cv::RotatedRect(p_temp[0], p_temp[1], p_temp[2]));
}

void sharpen(cv::Mat &img, cv::Mat &ret)
{
	ret.create(img.size(), img.type());
	int nc = img.channels();

	for (int j = 1; j < img.rows - 1; j++)
	{
		uchar* previous = img.ptr < uchar >(j - 1); // previous row
		uchar* current = img.ptr < uchar >(j);   // current row
		uchar* next = img.ptr < uchar >(j + 1);      // next row
		uchar* output = ret.ptr < uchar >(j);               //output row
		for (int i = nc; i < (img.cols - 1) * nc; i++)
		{
			*output++ = cv::saturate_cast < uchar > (5 * current[i] - current[i - nc] - current[i + nc] - previous[i] - next[i]);
		}
	}

	ret.row(0).setTo(cv::Scalar(0));
	ret.row(ret.rows - 1).setTo(cv::Scalar(0));
	ret.col(0).setTo(cv::Scalar(0));
	ret.col(ret.cols - 1).setTo(cv::Scalar(0));

}

int obj_find(cv::Mat& frame, cv::Mat mark)
{
	//����ͼ�����ݣ�����Ԥ����ͼ��ѡ��
	central_point = Point2f(-1.0, -1.0);
	frame.copyTo(img2);
	if (!img2.data)
	{
		cout << "error with coping frame!!!" << endl;
		return -3;
	}
	surf->detect(img2, keypoints2, mark);
	surf->compute(img2, keypoints2, descriptors2);
	//cv::drawKeypoints(img2, keypoints2, img2);
	matcher.match(descriptors1, descriptors2, matches);
	std::nth_element(matches.begin(), matches.begin() + 16, matches.end());
	std::cout << matches[0].distance << std::endl;
	std::cout << matches[10].distance << std::endl;
	std::cout << matches[16].distance << std::endl;
	for (i = 0; i < (int)matches.size(); i++)
	{
		if (matches[i].distance > thre) matches.erase(matches.begin() + i, matches.end());
	}
	std::nth_element(matches.begin(), matches.begin() + matches.size() / 2, matches.end());
	cout << matches.size() << endl;

	if (matches.size() > 16)
	{
		int a = 14;     //12
		matches.erase(matches.begin() + a + 1, matches.end());
		//std :: cout << matches [ a - 1 ].distance << std :: endl;

		for (i = 0; i < a; i++)
		{
			points2.push_back(keypoints2[matches[i].trainIdx].pt);
		}

		/*
		for ( i = 0; i < ( int ) points2.size (); i++ )
		{
		circle ( img2, points2 [ i ], 4, cv :: Scalar ( 255, 255, 255 ), CV_FILLED, CV_AA );
		}
		*/
		/*
		float disten;
		for (j = 0; j < (int)points2.size(); j++)
		{
		int m = 0;
		for (i = 0; i < (int)points2.size(); i++)
		{
		disten = sqrt(pow(points2[j].x - points2[i].x, 2) + pow(points2[j].y - points2[i].y, 2));
		if (disten > match_disten) //100
		{
		m++;
		}
		}
		if (m < (a / 2 - 1)) pot.push_back(points2[j]);
		}
		*/
		for (j = 0; j < (int)points2.size(); j++)
		{
			int m = 0;
			for (i = 0; i < (int)points2.size(); i++)
			{
				if ((fabs(points2[j].x - points2[i].x) < match_x) && (fabs(points2[j].y - points2[i].y) < match_y))
				{
					m++;
				}
			}
			if (m >= (a / 2)) pot.push_back(points2[j]);
		}
		if (pot.size() < 4) return 1;

		cv::RotatedRect box = createRotatedRect(cv::boundingRect(cv::Mat(pot)));//boundingRect
		Point2f vert[4];
		box.points(vert);
		for (i = 0; i < (int)pot.size(); i++)
		{
			circle(img2, pot[i], 3, cv::Scalar(255, 0, 0), CV_FILLED, CV_AA);
		}
		for (i = 0; i < 4; i++)
		{
			line(img2, vert[i], vert[(i + 1) % 4], cv::Scalar(0, 255, 255), 2, CV_AA);
		}
		central_point = box.center;
		circle(img2, central_point, 3, cv::Scalar(255, 255, 0), CV_FILLED, CV_AA);
		//std::cout << "object's coordinate is: ( " << central_point.x << ',' << central_point.y << " )" << std::endl;

		img2.copyTo(frame);
		if (!frame.data)
		{
			cout << "error with coping img2!!!" << endl;
			return -4;
		}

	}
	else if (matches.size() >= 8 && matches[(matches.size() - 1)].distance < 0.28)
	{
		for (i = 0; i < matches.size(); i++)
		{
			points2.push_back(keypoints2[matches[i].trainIdx].pt);
		}
		for (j = 0; j < (int)points2.size(); j++)
		{
			int m = 0;
			for (i = 0; i < (int)points2.size(); i++)
			{
				if ((fabs(points2[j].x - points2[i].x) < match_x) && (fabs(points2[j].y - points2[i].y) < match_y))
				{
					m++;
				}
			}
			if (m >= (matches.size() / 2)) pot.push_back(points2[j]);
		}
		if (pot.size() < 8) return 1;

		cv::RotatedRect box = createRotatedRect(cv::boundingRect(cv::Mat(pot)));//boundingRect
		Point2f vert[4];
		box.points(vert);
		for (i = 0; i < (int)pot.size(); i++)
		{
			circle(img2, pot[i], 3, cv::Scalar(255, 255, 0), CV_FILLED, CV_AA);
		}
		for (i = 0; i < 4; i++)
		{
			line(img2, vert[i], vert[(i + 1) % 4], cv::Scalar(255, 255, 255), 2, CV_AA);
		}
		central_point = box.center;
		circle(img2, central_point, 3, cv::Scalar(255, 255, 0), CV_FILLED, CV_AA);
		//std::cout << "object's coordinate is: ( " << central_point.x << ',' << central_point.y << " )" << std::endl;

		img2.copyTo(frame);
		if (!frame.data)
		{
			cout << "error with coping img2!!!" << endl;
			return -4;
		}
	}
	else if (matches.size() > 4 && matches[(matches.size() - 1)].distance < 0.30)
	{
		cv::Mat outmat;
		cv::drawMatches(img1, keypoints1, img2, keypoints2, matches, outmat, cv::Scalar(255, 255, 0));
		outmat.copyTo(frame);
	}

	keypoints2.clear();
	matches.clear();
	points2.clear();
	pot.clear();

	return 0;
}

bool tf_broadcast(tf::TransformBroadcaster &br,tf::Transform &transform,double x,double y,double z)
{
	transform.setOrigin(tf::Vector3(x,y,z));
	tf::Quaternion q;
	q.setRPY(0,0,0);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"zed_left_camera_frame","object"));
  return true;
}
