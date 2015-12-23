// Visualizer.cpp: define el punto de entrada de la aplicaci�n de consola.
#include "Utils/Viewer.h"
#include "Utils/Utilities.h"
#include "Utils/SimpleOpenNIViewer.h"
#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>


using namespace std;
using namespace boost::filesystem;

/*
Global Variables
*/
bool flag_color;
/*
Functions
*/
void printUsage(char* name)
{
	cout << "Name of program: " << name << endl
		<< "--file [path]					PCD and PLY file to be open" << endl
		<< "--folder [path]					All PCD and PLY Files at the folder" << endl
		<< "--save [path] [how many files]	Save a single PCD file" << endl
		<< "--show [path]					Show PCD" << endl
		<< "--print 						Print the Point Cloud" << endl
		<< "--kinect 						Show a Kinect pointcloud" << endl
		<< "-h								Print this usage" << endl;
}
int main(int argc, char** argv)
{

	if (pcl::console::find_argument(argc, argv, "-h") >= 0) {
		printUsage(argv[0]);
		return 0;
	}
	flag_color = false;
	if (pcl::console::find_argument(argc,argv,"--color") >= 0) {
		flag_color = true;
	}

	if(pcl::console::find_argument(argc,argv,"--kinect")>=0){
		SimpleOpenNIViewer v;
		v.run();
	}
	vector<string> paths;
	//read a file contain on a folder
	if (pcl::console::find_argument(argc, argv, "--folder") >= 0) {
		Utilities::getFiles(argv[pcl::console::find_argument(argc, argv, "--folder") + 1], paths);
	}
	//Read a file
	if (pcl::console::find_argument(argc, argv, "--file") >= 0) {
		vector<int> indices(pcl::console::parse_file_extension_argument(argc, argv, "pcd"));

		if (pcl::console::find_argument(argc, argv, "--save") >= 0){
			indices.erase(indices.end()-1);
		}

		Utilities::getFiles(argv, indices, paths);
		indices.clear();
		indices = pcl::console::parse_file_extension_argument(argc, argv, "ply");
		Utilities::getFiles(argv, indices, paths);
	}

	std::vector<pcl::PCLPointCloud2> clouds_blob(paths.size());
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_cloud_color(new pcl::PointCloud<pcl::PointXYZRGB>);

	Utilities::read(paths, clouds_blob);
	if (flag_color) {
		Utilities::convert2XYZRGB(clouds_blob, ptr_cloud_color);
	}
	else {
		Utilities::convert2XYZ(clouds_blob, ptr_cloud);
	}
	clouds_blob.clear();
	


	Eigen::Vector4f centroid;
	if (!flag_color) {

		if (pcl::console::find_argument(argc, argv, "--show") >= 0){
			if (ptr_cloud) Utilities::show(ptr_cloud);
		}

		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*ptr_cloud, centroid);
		pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.translation() << -centroid[0], -centroid[1], -centroid[2];
		pcl::transformPointCloud(*ptr_cloud, *transformed_cloud, transform);
		pcl::PointXYZ min;
		pcl::PointXYZ max;
		pcl::getMinMax3D(*transformed_cloud, min, max);
		ptr_cloud->clear();
		transform.translation() << 0, 0, -min.z;

		pcl::transformPointCloud(*transformed_cloud, *ptr_cloud, transform);
		if (pcl::console::find_argument(argc, argv, "--save") >= 0)
		{
			string path(argv[pcl::console::find_argument(argc, argv, "--save") + 1]);
			int how_many_files = atoi(argv[pcl::console::find_argument(argc, argv, "--save") + 2]);
			Utilities::writePCDFile(ptr_cloud, path, how_many_files);
		}
		if(pcl::console::find_argument(argc,argv,"--print") >= 0){
			for (size_t i = 0; i < ptr_cloud->points.size (); ++i)
				    std::cout << i << "	" << ptr_cloud->points[i].x
				              << " 	"    << ptr_cloud->points[i].y
				              << "	"    << ptr_cloud->points[i].z << std::endl;

		}
	}
	else 
	{
		if (pcl::console::find_argument(argc, argv, "--show") >= 0){
			if (ptr_cloud_color) Utilities::show(ptr_cloud_color, flag_color);
		}
		/*Compute the centroid*/
		pcl::compute3DCentroid(*ptr_cloud_color, centroid);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.translation() << -centroid[0], -centroid[1], -centroid[2];
		pcl::transformPointCloud(*ptr_cloud_color, *transformed_cloud, transform);
		ptr_cloud_color->clear();

		pcl::PointXYZRGB min;
		pcl::PointXYZRGB max;
		/*Search the minimun point on axis z*/
		pcl::getMinMax3D(*transformed_cloud, min, max);
		cout << min.x << " , " << min.y << " , " << min.z << endl;

		transform.translation() << 0, 0, -min.z;
		pcl::transformPointCloud(*transformed_cloud, *ptr_cloud_color, transform);
		if (pcl::console::find_argument(argc, argv, "--save") >= 0)
		{
			string path(argv[pcl::console::find_argument(argc, argv, "--save") + 1]);
			int how_many_files = atoi(argv[pcl::console::find_argument(argc, argv, "--save") + 2]);
			Utilities::writePCDFile(ptr_cloud_color, path, how_many_files);
		}
	}


    return 0;
}
