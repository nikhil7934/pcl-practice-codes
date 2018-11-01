/*
--------------------------------- 
Extracted from 
"http://pointclouds.org/documentation/tutorials/matrix_transform.php#matrix_transfrom"
and modified
---------------------------------
*/

#include<iostream>
#include<bits/stdc++.h>
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
  cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << "   Method{1 or 2}" << endl;
  cout << "\n\n Method for transformation \n 1 --> Using Matrix4f \n 2 --> Using Affine3f \n"<< endl;
  cout << "-h:  Show this help." << endl;
}
int main (int argc, char** argv)
{
  if(argc!=3)
  {
    showHelp(argv[0]);
    return 0;
  }
 	if(pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help"))
	{
    	showHelp(argv[0]);
    	return 0;
  }
  vector<int> filenames;
	bool file_is_pcd=false;
	int matmet = atoi(argv[1]);
	filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");
	printf("%d\n",pcl::console::find_switch(argc, argv, "1"));
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

/* Reminder: how transformation matrices work :

           |-------> This column is the translation
    | 1 0 0 x |  \
    | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
    | 0 0 1 z |  /
    | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

    METHOD #1: Using a Matrix4f
    This is the "manual" method, perfect to understand but error prone !
*/
  	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
// Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
  	float theta = M_PI/4; // The angle of rotation in radians
  	transform_1(0,0) = cos(theta);
 	transform_1(0,1) = -sin(theta);
  	transform_1(1,0) = sin(theta);
  	transform_1(1,1) = cos(theta);
// (row, column)
// Define a translation of 2.5 meters on the x axis.
  	transform_1(0,3) = 2.5;
  	transform_1(1,3) = 0.0;
  	transform_1(2,3) = 0.0;
  	if(matmet==1)
  	{
  		cout<<"theta for rotation:...> "<<endl;
  		cin>>theta;
  		cout<<"translation along X-axis..."<<endl;
  		cin>>transform_1(0,3);
  		cout<<"translation along Y-axis..."<<endl;
  		cin>>transform_1(1,3);
  		cout<<"translation along Z-axis..."<<endl;
  		cin>>transform_1(2,3);
  	}
// Print the transformation
  	printf ("Method #1: using a Matrix4f\n");
  	cout << transform_1 << endl;


/*  METHOD #2: Using a Affine3f
    This method is easier and less error prone
*/
  	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  	float thetaZ = M_PI/4;
  	float thetaY = 0.0;
  	float thetaX = 0.0;
  	float transX = 2.5;
  	float transY = 0.0;
  	float transZ = 0.0;
  	if(matmet==2)
  	{
  		cout<<"Translation along X-axis..."<<endl;
  		cin>>transX;
  		cout<<"Translation along Y-axis..."<<endl;
  		cin>>transY;
  		cout<<"Translation along Z-axis..."<<endl;
  		cin>>transZ;
  		cout<<"Rotation about X-axis..."<<endl;
  		cin>>thetaX;
  		cout<<"Rotation about Y-axis..."<<endl;
  		cin>>thetaY;
  		cout<<"Rotation about Z-axis..."<<endl;
  		cin>>thetaZ;	
  	}
// Define a translation of 2.5 meters on the x axis.
  	transform_2.translation() << transX, transY, transZ;
// The same rotation matrix as before; theta radians around Z axis
  	transform_2.rotate(Eigen::AngleAxisf(thetaZ, Eigen::Vector3f::UnitZ()));
  	transform_2.rotate(Eigen::AngleAxisf(thetaY, Eigen::Vector3f::UnitY()));
  	transform_2.rotate(Eigen::AngleAxisf(thetaX, Eigen::Vector3f::UnitX()));

// Print the transformation
  	printf ("\nMethod #2: using an Affine3f\n");
  	cout << transform_2.matrix() << endl;

// Executing the transformation
  	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ> ());
// You can either apply transform_1 or transform_2; they are the same
  	if(matmet==1)
  	{
  		pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_1);
  	}
  	else
  	{
  		pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_2);	
  	}

// Visualization
  	printf("\nPoint cloud colors: white = original point cloud\n"
    	   "                      red   = transformed point cloud\n");
  	pcl::visualization::PCLVisualizer viewer("Matrix transformation example");

// Define R,G,B colors for the point cloud
  	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(source_cloud, 255, 255, 255);
// We add the point cloud to the viewer and pass the color handler
  	viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");

  	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(transformed_cloud, 230, 20, 20); // Red
  	viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");
  	viewer.addCoordinateSystem (1.0, "cloud", 0);
// Setting background to a dark grey
  	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
  	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
//	viewer.setPosition(800, 400); // Setting visualiser window position

  	while(!viewer.wasStopped()) 
  	{ // Display the visualiser until 'q' key is pressed
    	viewer.spinOnce ();
  	}
  	return 0;
}