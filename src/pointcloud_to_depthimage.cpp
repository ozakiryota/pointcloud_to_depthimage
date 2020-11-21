#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class PointCloudToDepthImage{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscriber*/
		ros::Subscriber _sub_pc;
		/*publisher*/
		ros::Publisher _pub_img_64f;
		ros::Publisher _pub_img_8u;
		/*pc*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr _pc {new pcl::PointCloud<pcl::PointXYZ>};
		/*cv*/
		cv::Mat _img_cv_64f;
		cv::Mat _img_cv_8u;
		/*parameter*/
		int _num_rings;
		int _points_per_ring;
		double _fov_upper_deg;
		double _fov_lower_deg;
		double _max_range;

	public:
		PointCloudToDepthImage();
		void callbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		void pcToImage(void);
		void publication(std_msgs::Header header);
};

PointCloudToDepthImage::PointCloudToDepthImage()
	: _nhPrivate("~")
{
	std::cout << "--- pointcloud_to_depthimage ---" << std::endl;
	/*parameter*/
	_nhPrivate.param("num_rings", _num_rings, 32);
	std::cout << "_num_rings = " << _num_rings << std::endl;
	_nhPrivate.param("points_per_ring", _points_per_ring, 1092);
	std::cout << "_points_per_ring = " << _points_per_ring << std::endl;
	_nhPrivate.param("fov_upper_deg", _fov_upper_deg, 15.0);
	std::cout << "_fov_upper_deg = " << _fov_upper_deg << std::endl;
	_nhPrivate.param("fov_lower_deg", _fov_lower_deg, -25.0);
	std::cout << "_fov_lower_deg = " << _fov_lower_deg << std::endl;
	_nhPrivate.param("max_range", _max_range, 100.0);
	std::cout << "_max_range = " << _max_range << std::endl;
	/*sub*/
	_sub_pc = _nh.subscribe("/cloud", 1, &PointCloudToDepthImage::callbackPC, this);
	/*pub*/
	_pub_img_64f = _nh.advertise<sensor_msgs::Image>("/depth_image/64fc1", 1);
	_pub_img_8u = _nh.advertise<sensor_msgs::Image>("/depth_image/8uc1", 1);
}

void PointCloudToDepthImage::callbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	pcl::fromROSMsg(*msg, *_pc);
	pcToImage();
	publication(msg->header);
}

void PointCloudToDepthImage::pcToImage(void)
{
	/*reset*/
	// _img_cv_64f = cv::Mat::zeros(_num_rings, _points_per_ring, CV_64FC1);
	_img_cv_64f = cv::Mat(_num_rings, _points_per_ring, CV_64FC1, cv::Scalar(-1));
	/*resolution*/
	double angle_h_resolution = (_fov_upper_deg - _fov_lower_deg)/180.0*M_PI/(double)(_num_rings - 1);
	double angle_w_resolution = 2*M_PI/(double)_points_per_ring;
	/*input*/
	for(size_t i=0; i<_pc->points.size(); ++i){
		/*depth*/
		double depth = sqrt(_pc->points[i].x*_pc->points[i].x + _pc->points[i].y*_pc->points[i].y);
		/*row*/
		double angle_h = atan2(_pc->points[i].z, depth);
		int row = (_fov_upper_deg/180.0*M_PI - angle_h)/angle_h_resolution;
		if(row < 0 || row >= _num_rings){
			std::cout << "ERROR: row = " << row << std::endl;
			exit(1);
			// continue;
		}
		/*col*/
		double angle_w = atan2(_pc->points[i].y, _pc->points[i].x);
		int col = (_points_per_ring - 1) - (int)((angle_w + M_PI)/angle_w_resolution);
		if(col < 0 || col >= _points_per_ring){
			std::cout << "ERROR col" << std::endl;
			exit(1);
			// continue;
		}
		/*input*/
		_img_cv_64f.at<double>(row, col) = depth;
	}
	/*convert*/
	_img_cv_64f.convertTo(_img_cv_8u, CV_8UC1, 255/_max_range, 0);
}

void PointCloudToDepthImage::publication(std_msgs::Header header)
{
	sensor_msgs::ImagePtr img_ros_64f = cv_bridge::CvImage(header, "64FC1", _img_cv_64f).toImageMsg();
	sensor_msgs::ImagePtr img_ros_8u = cv_bridge::CvImage(header, "mono8", _img_cv_8u).toImageMsg();
	_pub_img_64f.publish(img_ros_64f);
	_pub_img_8u.publish(img_ros_8u);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_to_depthimage");
	
	PointCloudToDepthImage pointcloud_to_depthimage;

	ros::spin();
}
