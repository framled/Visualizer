// Visualizer.cpp: define el punto de entrada de la aplicación de consola.
//

#include "stdafx.h"
#include "Viewer.h"
#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

using namespace std;
using namespace boost::filesystem;
bool flag_color;
void printUsage(char* name);

void show(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
void show(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,string str);
void getFiles(string path, vector<string>& output);
void getFiles(char** argv, vector<int>& indices, vector<string>& output);

void readPCDFile(string path, pcl::PCLPointCloud2& cloud);
void readPLYFile(string path, pcl::PCLPointCloud2& cloud);


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
		getFiles(argv[pcl::console::find_argument(argc, argv, "--folder") + 1], paths);
	}
	
	if (pcl::console::find_argument(argc, argv, "--file") >= 0) {
		vector<int> indices(pcl::console::parse_file_extension_argument(argc, argv, "pcd"));
		getFiles(argv, indices, paths);
		indices.clear();
		indices = pcl::console::parse_file_extension_argument(argc, argv, "ply");
		getFiles(argv, indices, paths);
	}
	if (!flag_color) {
		vector<pcl::PCLPointCloud2> clouds_blob(paths.size());
		boost::thread_group tgroup;
		int i = 0;
		for (vector<string>::iterator path = paths.begin(); path != paths.end(); ++path) {
			size_t found_pcd = path->find(".pcd");
			size_t found_ply = path->find(".ply");

			if (found_pcd != string::npos) 
			{
				tgroup.create_thread(boost::bind(readPCDFile, *path, boost::ref(clouds_blob[i])));
			}
			else if(found_ply != string::npos)
			{
				tgroup.create_thread(boost::bind(readPLYFile, *path, boost::ref(clouds_blob[i])));
			}
			else {
				return 0;
			}
			i++;
		}
		tgroup.join_all();

		pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		for (vector<pcl::PCLPointCloud2>::iterator cloud = clouds_blob.begin(); cloud != clouds_blob.end(); ++cloud) {
			pcl::PointCloud<pcl::PointXYZ> c;
			pcl::fromPCLPointCloud2(*cloud, c);
			*ptr_cloud = *ptr_cloud + c;
		}
		
		if (pcl::console::find_argument(argc, argv, "--show") >= 0) 
			if (ptr_cloud) show(ptr_cloud);
		
		if (pcl::console::find_argument(argc, argv, "--save") >= 0) 
		{
			string path(argv[pcl::console::find_argument(argc, argv, "--save") + 1]);
			cout << "writing file " << path << endl;
			pcl::io::savePCDFileASCII(path, *ptr_cloud);
		}
	}
	else 
	{
		vector<pcl::PCLPointCloud2> clouds_blob(paths.size());
		boost::thread_group tgroup;
		int i = 0;
		for (vector<string>::iterator path = paths.begin(); path != paths.end(); ++path) {
			size_t found_pcd = path->find(".pcd");
			size_t found_ply = path->find(".ply");
			if (found_pcd != string::npos)
			{
				tgroup.create_thread(boost::bind(readPCDFile, *path, clouds_blob[i]));
			}
			else if (found_ply != string::npos)
			{
				tgroup.create_thread(boost::bind(readPLYFile, *path, clouds_blob[i]));
			}
			else {
				return 0;
			}
			/*pcl::PointCloud<pcl::PointXYZ> c;
			pcl::fromPCLPointCloud2(cloud_blob, c);
			*ptr_cloud = *ptr_cloud + c;*/
			i++;
		}
		tgroup.join_all();
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (vector<pcl::PCLPointCloud2>::iterator cloud = clouds_blob.begin(); 
			cloud != clouds_blob.end(); 
			++cloud) 
		{
			pcl::PointCloud<pcl::PointXYZRGB> c;
			pcl::fromPCLPointCloud2(*cloud, c);
			*ptr_cloud = *ptr_cloud + c;
		}
		
		if (pcl::console::find_argument(argc, argv, "--show") >= 0)
			if (ptr_cloud) show(ptr_cloud,"");

		if (pcl::console::find_argument(argc, argv, "--save") >= 0)
		{
			string path(argv[pcl::console::find_argument(argc, argv, "--save") + 1]);
			cout << "writing file " << path << endl;
			pcl::io::savePCDFileASCII(path, *ptr_cloud);
		}
	}
	system("PAUSE");
    return 0;
}

void printUsage(char* name) 
{
	cout << "Name of program: " << name << endl
		<< "--file [path]				PCD and PLY file to be open" << endl
		<< "--folder [path]				All PCD and PLY Files at the folder" << endl
		<< "--save [path]				Save a single PCD file" << endl
		<< "--show [path]				Show PCD" << endl
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

void getFiles(string path, vector<string>& output) 
{
	boost::filesystem::path p(path);
	for (auto i = directory_iterator(p); i != directory_iterator(); i++) 
	{
		if (!is_directory(i->path()))
		{
			size_t found_pcd = i->path().filename().string().find(".pcd");
			size_t found_ply = i->path().filename().string().find(".ply");
			if (found_pcd != string::npos || found_ply !=string::npos) 
			{
				cout << "Found file " << i->path().string() << endl;
				output.push_back(i->path().string());
			}
		}
	}
}
void getFiles(char** argv, vector<int>& indices, vector<string>& output)
{
	
	for (vector<int>::iterator i = indices.begin(); i != indices.end(); i++) 
	{
		cout << "Found file " << argv[*i] << endl;
		output.push_back(argv[*i]);
	}
}
void readPCDFile(string path, pcl::PCLPointCloud2& cloud)
{
	cout << "==============================================================" << endl
		<< "prepare for read " << path << endl;
	if (pcl::io::loadPCDFile(path, cloud) == -1)
	{
		string error("Couldn't read file " + path + " \n");
		PCL_ERROR(error.c_str());
	}
	else
	{
		cout << "success read file " << path << endl
			<< "==============================================================" << endl
			<< "Details of the point cloud " << endl
			<< "Width:	" << cloud.width << endl
			<< "Height: " << cloud.height << endl
			<< "==============================================================" << endl;
	}
}
void readPLYFile(string path, pcl::PCLPointCloud2& cloud) 
{
	
	if (pcl::io::loadPLYFile(path, cloud) == -1) 
	{
		string error("Couldn't read file " + path + " \n");
		PCL_ERROR(error.c_str());
	}
	else 
	{
		cout << "success read file " << path << endl
			<< "==============================================================" << endl
			<< "Details of the point cloud " << endl
			<< "Width:	" << cloud.width << endl
			<< "Height: " << cloud.height << endl
			<< "==============================================================" << endl;
	}
}

