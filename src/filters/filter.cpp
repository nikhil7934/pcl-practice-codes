/*
--------------------------------- 
Extracted from 
"http://pointclouds.org/documentation/tutorials/passthrough.php#passthrough"
and modified
---------------------------------
*/
#include<iostream>
#include<pcl/point_types.h>
#include<pcl/filters/passthrough.h>
#include<pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include<pcl/point_cloud.h>
#include<pcl/console/parse.h>
#include<pcl/common/transforms.h>
#include<pcl/visualization/pcl_visualizer.h>
using namespace std;
void showHelp(char *program_name)
{
  cout << endl;
  cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << " "<< endl;
  cout << ""<< endl;
  cout << "-h:  Show this help." << endl;
}
int main (int argc, char** argv)
{
  if(pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help"))
  {
      showHelp(argv[0]);
      return 0;
  }
  vector<int> filenames;
  bool file_is_pcd=false;
  if(filenames.size()!=1)
  {
      filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
      if(filenames.size()!=1)
      {
        showHelp (argv[0]);
        return -1;
      } 
      else 
          file_is_pcd = true;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  if(file_is_pcd)
  {
    if(pcl::io::loadPCDFile(argv[filenames[0]], *source_cloud) < 0)
    {
      cout << "Error loading point cloud " << argv[filenames[0]] << endl << endl;
      showHelp (argv[0]);
      return -1;
    }
  } 
  else
  {
    if(pcl::io::loadPLYFile(argv[filenames[0]], *source_cloud) < 0)
    {
      cout << "Error loading point cloud " << argv[filenames[0]] << endl << endl;
      showHelp (argv[0]);
      return -1;
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>());

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (source_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

// Visualization
  printf("\nPoint cloud colors: white = original point cloud\n"
         "                      red   = filtered point cloud\n"
         "Only RED coloured points are outputed after filteration..\n");
  pcl::visualization::PCLVisualizer viewer("Point Cloud Filetering Exxample");

// Define R,G,B colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(source_cloud, 255, 255, 255);
// We add the point cloud to the viewer and pass the color handler
  viewer.addPointCloud(source_cloud, source_cloud_color_handler, "original_cloud");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> filter_cloud_color_handler(cloud_filtered, 230, 20, 20); // Red
  viewer.addPointCloud (cloud_filtered, filter_cloud_color_handler, "filtered_cloud");
  viewer.addCoordinateSystem (1.0, "cloud", 0);
// Setting background to a dark grey
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "filtered_cloud");
  viewer.setPosition(800, 400); // Setting visualiser window position

  while(!viewer.wasStopped()) 
  { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }

  return (0);
}