#include "stdafx.h"
#include "Utilities.h"

Utilities::Utilities()
{
}

Utilities::~Utilities()
{
}


int Utilities::UniqueNumber()
{
	static int current = 0;
	return ++current;
}

std::vector<int> Utilities::getIndices(const unsigned int length)
{
	std::vector<int> indices(length);
	std::generate_n(indices.begin(), length, UniqueNumber);
	return indices;
}

void Utilities::convert2XYZ(std::vector<pcl::PCLPointCloud2>& input, pcl::PointCloud<pcl::PointXYZ>::Ptr& output)
{
	for (std::vector<pcl::PCLPointCloud2>::iterator cloud = input.begin(); cloud != input.end(); ++cloud) {
		pcl::PointCloud<pcl::PointXYZ> c;
		pcl::fromPCLPointCloud2(*cloud, c);
		*output += c;
	}
}
void Utilities::convert2XYZRGB(std::vector<pcl::PCLPointCloud2>& input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output)
{
	for (std::vector<pcl::PCLPointCloud2>::iterator cloud = input.begin(); cloud != input.end(); ++cloud) {
		pcl::PointCloud<pcl::PointXYZRGB> c;
		pcl::fromPCLPointCloud2(*cloud, c);
		*output += c;
	}
}

void Utilities::show(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
	pcl::Viewer<pcl::PointXYZ> viewer;
	viewer.addCloud(cloud, false);
	viewer.run();
}

void Utilities::show(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud, bool with_color = true)
{
	pcl::Viewer<pcl::PointXYZRGB> viewer;
	viewer.addCloud(cloud, with_color);
	viewer.run();
}

void Utilities::getFiles(std::string path, std::vector<std::string>& output)
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

void Utilities::getFiles(char** argv, std::vector<int>& indices, std::vector<std::string>& output)
{

	for (std::vector<int>::iterator i = indices.begin(); i != indices.end(); i++)
	{
		std::cout << "Found file " << argv[*i] << std::endl;
		output.push_back(argv[*i]);
	}
}

void Utilities::read(std::vector<std::string> paths, std::vector<pcl::PCLPointCloud2>& clouds_blob)
{
	boost::thread_group tgroup;
	int i = 0;
	for (std::vector<std::string>::iterator path = paths.begin(); path != paths.end(); ++path) {
		size_t found_pcd = path->find(".pcd");
		size_t found_ply = path->find(".ply");

		if (found_pcd != std::string::npos)
		{
			tgroup.create_thread(boost::bind(Utilities::readPCDFile, *path, boost::ref(clouds_blob[i])));
		}
		else if (found_ply != std::string::npos)
		{
			tgroup.create_thread(boost::bind(Utilities::readPLYFile, *path, boost::ref(clouds_blob[i])));
		}
		i++;
	}
	tgroup.join_all();
}

void Utilities::readPCDFile(std::string path, pcl::PCLPointCloud2& cloud)
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

void Utilities::readPLYFile(std::string path, pcl::PCLPointCloud2& cloud)
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


void Utilities::writePCDFile(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, std::string path, int how_many_files = 1)
{
	std::cout << "writing file " << path << std::endl;
	for (int i = 0; i < how_many_files; i++) {
		std::string extension(getExtension(path));
		std::string str(path);
		str.replace(str.find(extension), sizeof(extension) - 1, "_" + std::to_string(i) + extension);
		std::vector<int> indices(getIndices((cloud->width * cloud->height) / 4));
		pcl::io::savePCDFile(str, *cloud, indices);
	}
}
void Utilities::writePCDFile(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud, std::string path, int how_many_files = 1)
{
	std::cout << "writing file " << path << std::endl;
	for (int i = 0; i < how_many_files; i++) {
		std::string extension(getExtension(path));
		std::string str(path);
		str.replace(str.find(extension), sizeof(extension) - 1, "_" + std::to_string(i) + extension);
		std::vector<int> indices(getIndices((cloud->width * cloud->height) / 4));
		pcl::io::savePCDFile(str, *cloud, indices);
	}
}


std::string Utilities::getExtension(std::string file)
{
	std::string extension("");
	for (int i = file.length() - 1; i >= 0; i--) {
		if (file[i] != '.') 
		{
			extension += file[i];
		}
		else 
		{
			break;
		}
	}
	std::reverse(extension.begin(), extension.end());
	return "." + extension;
}

