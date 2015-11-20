#ifndef SOURCE_DIRECTORY__VIEWER_H_
#define SOURCE_DIRECTORY__VIEWER_H_
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <iostream>

namespace pcl {
	template<typename PointT>
	class Viewer
	{
	public:
		typedef pcl::PointCloud<PointT> PointCloud;
		typedef typename PointCloud::ConstPtr ConstPtr;
		Viewer();
		void run();
		void addCloud(const ConstPtr& cloud, bool isColored);
		virtual ~Viewer();
	private:
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
		void keyboard_callback(const pcl::visualization::KeyboardEvent& event, void*);
		void mouse_callback(const pcl::visualization::MouseEvent& event, void*);
	};
}

#endif /*SOURCE_DIRECTORY__UTILITIES_H_*/
