//
//#include <QApplication>
//
//#include "main_window.h"
//
//int main(int argc, char *argv[])
//{
//  // Make Xlib and GLX thread safe under X11
//  QApplication::setAttribute(Qt::AA_X11InitThreads);
//  QApplication application(argc, argv);
//
//  MainWindow main_window;
//  main_window.showMaximized();
//
//  return application.exec();
//}

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>



int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("points.pcd", *cloud) == -1) //* load the file
    {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
    }

        // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);

    // Compute the features
    ne.compute (*cloud_normals);

    // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*

    return 0;
}