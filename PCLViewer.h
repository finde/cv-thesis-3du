
#include <pcl/visualization/cloud_viewer.h>

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
boost::shared_ptr<pcl::visualization::PCLVisualizer> meshVis(pcl::PolygonMesh mesh);
boost::shared_ptr<pcl::visualization::PCLVisualizer> textureVis(pcl::TextureMesh mesh);