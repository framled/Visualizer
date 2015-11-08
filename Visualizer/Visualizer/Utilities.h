#pragma once
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include "Viewer.h"



class Utilities
{
public:
	Utilities();
	static void show(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
	static void show(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud, bool with_color);
	static void getFiles(std::string path, std::vector<std::string>& output);
	static void getFiles(char** argv, std::vector<int>& indices, std::vector<std::string>& output);
	static void read(std::vector<std::string> paths, std::vector<pcl::PCLPointCloud2>& clouds_blob);
	static void readPCDFile(std::string path, pcl::PCLPointCloud2& cloud);
	static void readPLYFile(std::string path, pcl::PCLPointCloud2& cloud);
	static void writePCDFile(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&, std::string path, int how_many_files);
	static void writePCDFile(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&, std::string path, int how_many_files);
	static std::string getExtension(std::string file);
	static int UniqueNumber();
	static std::vector<int> getIndices(const unsigned int length);
	static void Utilities::convert2XYZ(std::vector<pcl::PCLPointCloud2>& input, pcl::PointCloud<pcl::PointXYZ>::Ptr& output);
	static void Utilities::convert2XYZRGB(std::vector<pcl::PCLPointCloud2>& input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output);
	~Utilities();
};

