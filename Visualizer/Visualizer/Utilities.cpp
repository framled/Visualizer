#include "Utilities.h"

Utilities::Utilities()
{
}


Utilities::~Utilities()
{
}


void printUsage(char* name)
{
	std::cout << "Name of program: " << name << std::endl
		<< "--file [path]				PCD and PLY file to be open" << std::endl
		<< "--folder [path]				All PCD and PLY Files at the folder" << std::endl
		<< "--save [path]				Save a single PCD file" << std::endl
		<< "--show [path]				Show PCD" << std::endl
		<< "-h							Print this usage" << std::endl;
}
void show(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
	pcl::Viewer<pcl::PointXYZ> viewer;
	viewer.addCloud(cloud, false);
	viewer.run();
}
void showColor(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud, bool with_color = true)
{
	pcl::Viewer<pcl::PointXYZRGB> viewer;
	viewer.addCloud(cloud, with_color);
	viewer.run();
}

void getFiles(std::string path, std::vector<std::string>& output)
{
	boost::filesystem::path p(path);
	for (auto i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++)
	{
		if (!is_directory(i->path()))
		{
			size_t found_pcd = i->path().filename().string().find(".pcd");
			size_t found_ply = i->path().filename().string().find(".ply");
			if (found_pcd != std::string::npos || found_ply != std::string::npos)
			{
				std::cout << "Found file " << i->path().string() << std::endl;
				output.push_back(i->path().string());
			}
		}
	}
}
void getFiles(char** argv, std::vector<int>& indices, std::vector<std::string>& output)
{

	for (std::vector<int>::iterator i = indices.begin(); i != indices.end(); i++)
	{
		std::cout << "Found file " << argv[*i] << std::endl;
		output.push_back(argv[*i]);
	}
}
void readPCDFile(std::string path, pcl::PCLPointCloud2& cloud)
{
	std::cout << "==============================================================" << std::endl
		<< "prepare for read " << path << std::endl;
	if (pcl::io::loadPCDFile(path, cloud) == -1)
	{
		std::string error("Couldn't read file " + path + " \n");
		PCL_ERROR(error.c_str());
	}
	else
	{
		std::cout << "success read file " << path << std::endl
			<< "==============================================================" << std::endl
			<< "Details of the point cloud " << std::endl
			<< "Width:	" << cloud.width << std::endl
			<< "Height: " << cloud.height << std::endl
			<< "==============================================================" << std::endl;
	}
}
void readPLYFile(std::string path, pcl::PCLPointCloud2& cloud)
{

	if (pcl::io::loadPLYFile(path, cloud) == -1)
	{
		std::string error("Couldn't read file " + path + " \n");
		PCL_ERROR(error.c_str());
	}
	else
	{
		std::cout << "success read file " << path << std::endl
			<< "==============================================================" << std::endl
			<< "Details of the point cloud " << std::endl
			<< "Width:	" << cloud.width << std::endl
			<< "Height: " << cloud.height << std::endl
			<< "==============================================================" << std::endl;
	}
}
