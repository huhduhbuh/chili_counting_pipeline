#include <iostream>

#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include <string>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

int

main (int argc, char** argv)

{


  std::ifstream f("config.json");

    json config;
    f >> config;

   if (argc != 2)

  {

    pcl::console::print_error ("Syntax is: %s <pcd-file> \n ", argv[0]);

    return (1);

  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);


  // Fill in the cloud data

  pcl::PCDReader reader;

  // Replace the path below with the path where you saved your file

  reader.read<pcl::PointXYZ> (argv[1], *cloud);


  std::cerr << "Cloud before filtering: " << std::endl;

  std::cerr << *cloud << std::endl;


  // Create the filtering object

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

  sor.setInputCloud (cloud);

  sor.setMeanK (config["sor"]["meanK"]);

  sor.setStddevMulThresh (config["sor"]["StddevMultThresh"]);

  sor.filter (*cloud_filtered);


  std::cerr << "Cloud after filtering: " << std::endl;

  std::cerr << *cloud_filtered << std::endl;


  pcl::PCDWriter writer;

  std::string filename(argv[1]);
  size_t last_dot = filename.find_last_of('.');
    
    // If a dot was found, take the substring before it
  std::string basename = (last_dot == std::string::npos) ? filename : filename.substr(0, last_dot);

  std::string inliers_name = basename + "_inliers.pcd";

  std::cout << inliers_name << std::endl;

  writer.write<pcl::PointXYZ> (inliers_name, *cloud_filtered, false);


  sor.setNegative (true);

  sor.filter (*cloud_filtered);

  //std::string out_name = argv[1] + "_outliers.pcd"

  //writer.write<pcl::PointXYZ> ("outliers.pcd", *cloud_filtered, false);


  return (0);

}