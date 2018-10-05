#include<iostream>
// io/pcd_io.h contains all types of definitions of pcd files input/output operations....
#include<pcl/io/pcd_io.h>
// point_types.h contains all definitions of PointT type Structures. 
// Example: pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointXYZRGB> etc.
#include<pcl/point_types.h>

using namespace std;

int main()
{
	// First creating a PointCloud data structure.
	pcl::PointCloud<pcl::PointXYZ> pcloud;
	// Now add arritributes for PointCloud with different attributes.
	pcloud.width    = 1000;
  	pcloud.height   = 1;
  	pcloud.is_dense = true;


  	pcloud.points.resize(pcloud.width * pcloud.height);

  	for (size_t i = 0; i < pcloud.points.size (); ++i)
  	{
    	pcloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    	pcloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    	pcloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  	}

  	// This saves the above PointCloud data into File....
  	pcl::io::savePCDFileASCII ("test_pcd.pcd", pcloud);

  	cerr << "Saved " << pcloud.points.size () << " data points to test_pcd.pcd." << endl;

  	for (size_t i = 0; i < pcloud.points.size (); ++i)
    	cerr << "    " << pcloud.points[i].x << " " << pcloud.points[i].y << " " << pcloud.points[i].z << endl;

	return 0;
} 