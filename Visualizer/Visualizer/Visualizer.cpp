// Visualizer.cpp: define el punto de entrada de la aplicación de consola.

#include "stdafx.h"
#include "Viewer.h"
#include "Utilities.h"
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
		<< "--file [path]				PCD and PLY file to be open" << endl
		<< "--folder [path]				All PCD and PLY Files at the folder" << endl
		<< "--save [path]				Save a single PCD file" << endl
		<< "--show [path]				Show PCD" << endl
		<< "-h							Print this usage" << endl;
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
	vector<string> paths;
	if (pcl::console::find_argument(argc, argv, "--folder") >= 0) {
		Utilities::getFiles(argv[pcl::console::find_argument(argc, argv, "--folder") + 1], paths);
	}
	
	if (pcl::console::find_argument(argc, argv, "--file") >= 0) {
		vector<int> indices(pcl::console::parse_file_extension_argument(argc, argv, "pcd"));
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

	
	Eigen::Vector4f centroid;
	if (!flag_color) {

		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*ptr_cloud, centroid);
		pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.translation() << -centroid[0], -centroid[1], -centroid[2];
		pcl::transformPointCloud(*ptr_cloud, *transformed_cloud, transform);
		pcl::PointXYZ min;
		pcl::PointXYZ max;
		pcl::getMinMax3D(*ptr_cloud, min, max);

		transform.translation() << 0, -min.y, 0;
		pcl::transformPointCloud(*ptr_cloud, *transformed_cloud, transform);

		if (pcl::console::find_argument(argc, argv, "--show") >= 0)
			if (transformed_cloud) Utilities::show(transformed_cloud);

		if (pcl::console::find_argument(argc, argv, "--save") >= 0)
		{
			string path(argv[pcl::console::find_argument(argc, argv, "--save") + 1]);
			int how_many_files = atoi(argv[pcl::console::find_argument(argc, argv, "--save") + 2]);
			Utilities::writePCDFile(transformed_cloud, path, how_many_files);
		}
	}
	else 
	{
		
		pcl::compute3DCentroid(*ptr_cloud_color, centroid);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.translation() << -centroid[0], -centroid[1], -centroid[2];
		pcl::transformPointCloud(*ptr_cloud_color, *transformed_cloud, transform);

		pcl::PointXYZRGB min;
		pcl::PointXYZRGB max;
		
		pcl::getMinMax3D(*ptr_cloud_color, min, max);

		if (pcl::console::find_argument(argc, argv, "--show") >= 0)
			if (ptr_cloud) Utilities::show(ptr_cloud_color, flag_color);


		if (pcl::console::find_argument(argc, argv, "--save") >= 0)
		{

			string path(argv[pcl::console::find_argument(argc, argv, "--save") + 1]);
			int how_many_files = atoi(argv[pcl::console::find_argument(argc, argv, "--save") + 2]);
			Utilities::writePCDFile(ptr_cloud_color, path, how_many_files);
		}
	}
	
	system("PAUSE");
    return 0;
}