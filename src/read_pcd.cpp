#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>

using namespace std;

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud (new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../samples/test_pcd_01.pcd", *pcloud) == -1)
	{
    	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    	return (-1);
  	}
  	cout << "Loaded "
            << pcloud->width * pcloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << endl;
  	for (size_t i = 0; i < pcloud->points.size (); ++i)
    	cout << "    " << pcloud->points[i].x
              	<< " "    << pcloud->points[i].y
              	<< " "    << pcloud->points[i].z << endl;
	return 0;
}