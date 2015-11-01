// Visualizer.cpp: define el punto de entrada de la aplicación de consola.
//

#include "stdafx.h"
#include "Viewer.h"
#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

using namespace std;

bool flag_color;
void printUsage(char* name);

void show(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
void show(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,string str);


pcl::PCLPointCloud2 readPCDFile(string path);
pcl::PCLPointCloud2 readPLYFile(string path);

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
	
	
	if (pcl::console::find_argument(argc, argv, "--file") >= 0) {
		pcl::PCLPointCloud2 cloud_blob;
		
		if (!flag_color) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			vector<int> indices(pcl::console::parse_file_extension_argument(argc, argv, "pcd"));
			for (int i = 0; i < indices.size(); i++) {
				string path(argv[indices[i]]);
				cloud_blob = readPCDFile(path);
				pcl::PointCloud<pcl::PointXYZ> c;
				pcl::fromPCLPointCloud2(cloud_blob, c);
				*ptr_cloud = *ptr_cloud + c;
				

			}
			indices.clear();
			indices = pcl::console::parse_file_extension_argument(argc, argv, "ply");
			for (int i = 0; i < indices.size(); i++) {
				string path(argv[indices[i]]);
				cloud_blob = readPLYFile(path);
				pcl::PointCloud<pcl::PointXYZ> c;
				pcl::fromPCLPointCloud2(cloud_blob, c);
				*ptr_cloud = *ptr_cloud + c;
			}
			if (ptr_cloud) {
				show(ptr_cloud);
			}
		}
		else 
		{
			pcl::PointCloud<pcl::PointXYZRGB> cloud;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_cloud(&cloud);
			vector<int> indices(pcl::console::parse_file_extension_argument(argc, argv, "pcd"));
			for (int i = 0; i < indices.size(); i++) {
				string path(argv[indices[i]]);
				cloud_blob = readPCDFile(path);
				pcl::PointCloud<pcl::PointXYZRGB> c;
				pcl::fromPCLPointCloud2(cloud_blob, c);
				cloud = cloud + c;

			}
			indices.clear();
			indices = pcl::console::parse_file_extension_argument(argc, argv, "ply");
			for (int i = 0; i < indices.size(); i++) {
				string path(argv[indices[i]]);
				cloud_blob = readPLYFile(path);
				pcl::PointCloud<pcl::PointXYZRGB> c;
				pcl::fromPCLPointCloud2(cloud_blob, c);
				cloud = cloud + c;
			}
			if (ptr_cloud) {
				show(ptr_cloud, "color");
			}
			
		}
		
	}
    return 0;
}

void printUsage(char* name) 
{
	cout << "Name of program: " << name << endl
		<< "--file [path]				PCD file to be open" << endl
		<< "-h							Print this usage" << endl;
}
void show(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
	pcl::Viewer<pcl::PointXYZ> viewer;
	viewer.addCloud(cloud, flag_color);
	viewer.run();
}
void show(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,string str)
{
	pcl::Viewer<pcl::PointXYZRGB> viewer;
	viewer.addCloud(cloud, flag_color);
	viewer.run();
}
pcl::PCLPointCloud2 readPCDFile(string path) {
	cout << "prepare for read " << path << endl;
	pcl::PCLPointCloud2 cloud;
	if (pcl::io::loadPCDFile(path, cloud) == -1) {
		string error("Couldn't read file " + path + " \n");
		PCL_ERROR(error.c_str());
	}
	else 
	{
		cout << "success read file " << path << endl;
	}

	return cloud;
}
pcl::PCLPointCloud2 readPLYFile(string path) {
	pcl::PCLPointCloud2 cloud;
	if (pcl::io::loadPLYFile(path, cloud) == -1) {
		string error("Couldn't read file " + path + " \n");
		PCL_ERROR(error.c_str());
	}
	else {
		cout << "success read file " << path << endl;
	}
	return cloud;
}