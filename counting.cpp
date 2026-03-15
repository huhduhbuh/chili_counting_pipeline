#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/lccp_segmentation.h>
#include <pcl/common/pca.h>
#include <pcl/common/io.h>
//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>
#include <algorithm>
#include <cmath>
#include <vector>
#include <random>
#include <nlohmann/json.hpp>
#include <string>
#include <cstdlib>
#include <ctime> 

using json = nlohmann::json;

// Types
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;

struct Edit {
    enum Type { Split, Merge, Delete } type;
    std::uint32_t cluster_id;                // main cluster affected
    std::uint32_t other_id;    // e.g. for merges
};

void addSupervoxelConnectionsToViewer (PointT &supervoxel_center,

                                       PointCloudT &adjacent_supervoxel_centers,

                                       std::string supervoxel_name,

                                       pcl::visualization::PCLVisualizer::Ptr & viewer);
float normalizeWithRange(float value, std::pair<float,float> ideal_range) ;
uint32_t getNearbySmallCluster(const std::map<std::uint32_t, std::vector<float>>& c_features, uint32_t id, float dist_threshold, std::vector<uint32_t> tagged) ;
inline void hsv2rgb(float h, float s, float v, uint8_t &r, uint8_t &g, uint8_t &b) ;

int main (int argc, char ** argv)
{
  std::ifstream f(argv[2]);
  json config;
  f >> config;

  if (argc < 3)
  {
    pcl::console::print_error ("Syntax is: %s <pcd-file> \n "
                                "--NT Dsables the single cloud transform \n"
                                "-v <voxel resolution>\n-s <seed resolution>\n"
                                "-c <color weight> \n-z <spatial weight> \n"
                                "-k <k factor>\n", 
                                "-ct <concavity threshold>\n", 
                                "-st <smoothness threshold>\n", 
                                "-ms <min segment>\n", 
                                "-ec <extended convexity>\n", 
                                "-sc <sanity criterion>\n", 
                                argv[0]);
    return (1);
  }

  PointCloudT::Ptr cloud (new PointCloudT);
  pcl::console::print_highlight ("Loading point cloud...\n");

  if (pcl::io::loadPCDFile<PointT> (argv[1], *cloud))
  {
    pcl::console::print_error ("Error loading cloud file!\n");
    return (1);
  }

  //SUPERVOXEL VARS

  cout << "cloud size: " << cloud->size() << endl;
  bool disable_transform = pcl::console::find_switch (argc, argv, "--NT");
  
  float voxel_resolution = config["counting"]["general"]["voxel_res"];
  bool voxel_res_specified = pcl::console::find_switch (argc, argv, "-v");
  if (voxel_res_specified)
    pcl::console::parse (argc, argv, "-v", voxel_resolution);

  float seed_resolution = config["counting"]["general"]["seed_res"];
  bool seed_res_specified = pcl::console::find_switch (argc, argv, "-s");
  if (seed_res_specified)
    pcl::console::parse (argc, argv, "-s", seed_resolution);

  float color_importance = config["counting"]["lccp1"]["color"];
  if (pcl::console::find_switch (argc, argv, "-c"))
    pcl::console::parse (argc, argv, "-c", color_importance);

  float spatial_importance = config["counting"]["lccp1"]["space"];
  if (pcl::console::find_switch (argc, argv, "-z"))
    pcl::console::parse (argc, argv, "-z", spatial_importance);

  float normal_importance = config["counting"]["lccp1"]["normal"];
  if (pcl::console::find_switch (argc, argv, "-n"))
    pcl::console::parse (argc, argv, "-n", normal_importance);


  // LCCP VARS

  unsigned int k_factor = 0;
  if (pcl::console::find_switch (argc, argv, "-k"))
    pcl::console::parse (argc, argv, "-k", k_factor);

  float concavity_tolerance_threshold = config["counting"]["lccp1"]["ct"];
    if (pcl::console::find_switch (argc, argv, "-ct"))
    pcl::console::parse (argc, argv, "-ct", concavity_tolerance_threshold);	
  
  float smoothness_threshold = config["counting"]["lccp1"]["st"];
  if (pcl::console::find_switch (argc, argv, "-st"))
    pcl::console::parse (argc, argv, "-st", smoothness_threshold);

	uint32_t min_segment_size = 0;
    if (pcl::console::find_switch (argc, argv, "-ms"))
    pcl::console::parse (argc, argv, "-n", min_segment_size);

	bool use_extended_convexity = false;
    if (pcl::console::find_switch (argc, argv, "-ec"))
    pcl::console::parse (argc, argv, "-ec", use_extended_convexity);

	bool use_sanity_criterion = false;
    if (pcl::console::find_switch (argc, argv, "-sc"))
    pcl::console::parse (argc, argv, "-sc", use_sanity_criterion);


  //////////////////////////////  //////////////////////////////

  ////// This is how to use supervoxels

  //////////////////////////////  //////////////////////////////


  pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution);
  if (disable_transform)
    super.setUseSingleCameraTransform (false);

  super.setInputCloud (cloud);
  super.setColorImportance (color_importance);
  super.setSpatialImportance (spatial_importance);
  super.setNormalImportance (normal_importance);

  std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;

  pcl::console::print_highlight ("Extracting supervoxels!\n");
  super.extract (supervoxel_clusters);
  pcl::console::print_info ("Found %d supervoxels\n", supervoxel_clusters.size ());

  //VISUALIZER 
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  PointCloudT::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud ();
  viewer->addPointCloud (voxel_centroid_cloud, "voxel centroids");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2.0, "voxel centroids");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.95, "voxel centroids");

  PointLCloudT::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud ();
  viewer->addPointCloud (labeled_voxel_cloud, "labeled voxels");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "labeled voxels");
  PointNCloudT::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud (supervoxel_clusters);

  //We have this disabled so graph is easy to see, uncomment to see supervoxel normals
  //viewer->addPointCloudNormals<PointNormal> (sv_normal_cloud,1,0.05f, "supervoxel_normals");

  pcl::console::print_highlight ("Getting supervoxel adjacency\n");
  std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency;
  super.getSupervoxelAdjacency (supervoxel_adjacency);
  //To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
  for (auto label_itr = supervoxel_adjacency.cbegin (); label_itr != supervoxel_adjacency.cend (); )
  {
    //First get the label
    std::uint32_t supervoxel_label = label_itr->first;

    //Now get the supervoxel corresponding to the label
    pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);

    //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
    PointCloudT adjacent_supervoxel_centers;

    for (auto adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first; adjacent_itr!=supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr)
    {
      pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel = supervoxel_clusters.at (adjacent_itr->second);
      adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);
    }

    //Now we make a name for this polygon
    std::stringstream ss;
    ss << "supervoxel_" << supervoxel_label;

    //This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
    addSupervoxelConnectionsToViewer (supervoxel->centroid_, adjacent_supervoxel_centers, ss.str (), viewer);

    //Move iterator forward to next label
    label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
  }

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
  }

  //////////////////////////////  //////////////////////////////

  ////// This is how to use LCCP

  //////////////////////////////  //////////////////////////////

	PCL_INFO("Starting Segmentation\n");
	pcl::LCCPSegmentation<PointT> lccp;
	lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
	lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
	lccp.setKFactor(k_factor);
	lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
	lccp.setMinSegmentSize(min_segment_size);
	lccp.segment();
	
	PCL_INFO("Interpolation voxel cloud -> input cloud and relabeling\n");
 
	pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();
	pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared();
	lccp.relabelCloud(*lccp_labeled_cloud);
	//pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList sv_adjacency_list;
	//lccp.getSVAdjacencyList(sv_adjacency_list);
  //std::map<std::uint32_t, std::set<std::uint32_t>> segment_map;
  //lccp.getSegmentToSupervoxelMap(segment_map);
  //cout << "map size: " << segment_map.size() << endl; 

  // Configure Visualizer
  pcl::visualization::PCLVisualizer::Ptr viewer2 (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  // pcl::visualization::PCLVisualizer::Ptr viewer2 = pcl::visualization::PCLVisualizer ("3D Viewer",false);
  viewer2->addPointCloud (lccp_labeled_cloud, "Segmented point cloud");

	/*vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow = viewer2.getRenderWindow();
  qvtk->SetRenderWindow(renderWindow);
  viewer2.setupInteractor(qvtk->GetInteractor(),qvtk->GetRenderWindow());

  qvtk->update();
  qvtk->GetRenderWindow()->Render();
  //endTime = clock();
  //cout<<"The run time is :" << (double)(endTime-startTime)/CLOCKS_PER_SEC <<"s"<<endl;
  */

  while (!viewer2->wasStopped ())
  {
    viewer2->spinOnce (100);
  }


  //////////////////////////////  //////////////////////////////

  ////// WRITING SUPERVOXEL AND LCCP POINT CLOUDS TO FILE

  //////////////////////////////  //////////////////////////////

  pcl::PCDWriter writer;
  std::string filename(argv[1]);
  size_t last_dot = filename.find_last_of('.');
  // If a dot was found, take the substring before it
  std::string basename = (last_dot == std::string::npos) ? filename : filename.substr(0, last_dot);
  srand(time(0));

  // write supervoxel
  std::string super_name = basename + "_super.pcd";
	pcl::PointCloud<PointT>::Ptr sv_colored_cloud(new pcl::PointCloud<PointT>);
  for (const auto& pt : sv_labeled_cloud->points)
  {
    int r = (pt.label * 53) % 256;
    int g = (pt.label * 97) % 256;
    int b = (pt.label * 193) % 256; 
    
    PointT new_pt(pt.x, pt.y, pt.z, r, g, b, 1);
    sv_colored_cloud->points.push_back(new_pt);
  }
  sv_colored_cloud->width = sv_labeled_cloud->width;
  sv_colored_cloud->height = sv_labeled_cloud->height;
  writer.write<pcl::PointXYZRGBA> (super_name, *sv_colored_cloud, false);
 
  // write lccp 
  std::string lccp0_name = basename + "_lccp0.pcd";
  pcl::PointCloud<PointT>::Ptr lccp_colored_cloud(new pcl::PointCloud<PointT>);
  for (const auto& pt : lccp_labeled_cloud->points)
  {
    int r = (pt.label * 53) % 256;
    int g = (pt.label * 97) % 256;
    int b = (pt.label * 193) % 256; 
    
    PointT new_pt(pt.x, pt.y, pt.z, r, g, b, 1);
    lccp_colored_cloud->points.push_back(new_pt);
  }
  lccp_colored_cloud->width = lccp_labeled_cloud->width;
  lccp_colored_cloud->height = lccp_labeled_cloud->height;
  writer.write<PointT> (lccp0_name, *lccp_colored_cloud, false);


  //////////////////////////////  //////////////////////////////

  ////// GLOBAL FEATURES

  //////////////////////////////  //////////////////////////////

  printf("starting global features\n");

  // convert lccp output to map with labels -> point cloud versus labels -> supervoxel list
  /*std::map<std::uint32_t, pcl::PointCloud<PointT>::Ptr> global_map;

  for (const auto& pair : segment_map) {
      uint32_t label = pair.first;

      for (const auto& sv_id : pair.second) {
          auto sv = supervoxel_clusters.at(sv_id);  // lookup supervoxel by ID

          // allocate cloud if first time seeing this label
          if (!global_map[label]) {   // shared_ptr is null
              PointCloudT::Ptr blank_cloud (new PointCloudT);
              global_map[label] = blank_cloud;
          }

          // add all points from this supervoxel to the label’s cloud
          *(global_map[label]) += *(sv->voxels_);
      }
  }*/

  std::map<uint32_t, pcl::PointCloud<PointT>::Ptr> global_map;

  for (const auto& pt : lccp_labeled_cloud->points)
  {
      uint32_t label = pt.label;

      if (label == 0) continue;

      // allocate cloud if first time seeing this label
      if (!global_map.count(label))
      {
          global_map[label] = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
      }

      global_map[label]->points.emplace_back(pt.x, pt.y, pt.z);
  }
  cout << "lccp_labeled_cloud size: " << global_map.size() << endl;

  printf("finished converting lccp output\n");

  /*
  // for testing if points exist in global_map
  for (const auto& pair : global_map) {
      printf("label: %d\n", pair.first);
    for (const auto& point : pair.second->points) {
      printf("point: %lf\n", point.x);
    }
    printf("\n");
      
  }
  */

  // PARAMETERS FOR GLOBAL FEATURE EXTRACTION

  float w_length = config["counting"]["global_weights"]["w_length"];
  float w_cross = config["counting"]["global_weights"]["w_cross"];
  float w_volume = config["counting"]["global_weights"]["w_volume"];
  uint32_t k_iter = config["counting"]["ideals"]["k_iter"];

  std::pair<float,float> ideal_length = config["counting"]["ideals"]["length"];
  std::pair<float,float> ideal_cross= config["counting"]["ideals"]["cross"];
  std::pair<float,float> ideal_volume = config["counting"]["ideals"]["volume"]; //{3e-6f, 6e-6f};

  float merge_dist_threshold = ideal_length.first;


  std::vector<Edit> delete_edits;

  std::map<std::uint32_t, std::vector<float>> chili_features; // store global features per chili


  for (size_t i = 0; i < k_iter; i++) {
      chili_features.clear();

    
      // GET GLOBAL FEATURES PER CLUSTER
      for (const auto& pair : global_map) {
          uint32_t label = pair.first;
          pcl::PointCloud<PointT>::Ptr cluster_pointcloud = pair.second;

          float chili_length = 0.000000001;
          float chili_width = 0.000000001;
          float chili_height = 0.000000001;
          Eigen::Vector3f center(1, 1, 1);

          // PCA
          pcl::PCA<PointT>pca;
          pca.setInputCloud(cluster_pointcloud);


          if (cluster_pointcloud->size() >= 3) {

              float longest_eVal = pca.getEigenValues()[0];
              float mid_eVal = pca.getEigenValues()[1];
              float shortest_eVal = pca.getEigenValues()[2];

              printf("pca cluster\n");

              
              Eigen::RowVector3f longest_eVec = pca.getEigenVectors().col(0);
              Eigen::RowVector3f mid_eVec = pca.getEigenVectors().col(1);
              Eigen::RowVector3f shortest_eVec = pca.getEigenVectors().col(2);

              printf("eVals %d: %f %f %f\n", label, longest_eVal, mid_eVal, shortest_eVal);



              // length 
              center = Eigen::Vector3f(pca.getMean()[0], pca.getMean()[1], pca.getMean()[2]);

              float min_proj = std::numeric_limits<float>::max();
              float max_proj = std::numeric_limits<float>::lowest();

              for (const auto& pt : cluster_pointcloud->points) {
                  Eigen::Vector3f p(pt.x, pt.y, pt.z);
                  float proj = longest_eVec.dot(p - center); // project onto major axis
                  min_proj = std::min(min_proj, proj);
                  max_proj = std::max(max_proj, proj);
              }

              chili_length = max_proj - min_proj;
              std::cout << "Estimated chili length: " << chili_length << std::endl;





              // cross section ratio
              // width
              min_proj = std::numeric_limits<float>::max();
              max_proj = std::numeric_limits<float>::lowest();
              for (const auto& pt : cluster_pointcloud->points) {
                  Eigen::Vector3f p(pt.x, pt.y, pt.z);
                  float proj = mid_eVec.dot(p - center); // project onto major axis
                  min_proj = std::min(min_proj, proj);
                  max_proj = std::max(max_proj, proj);
              }
              chili_width = max_proj - min_proj;
              std::cout << "Estimated chili width: " << chili_width << std::endl;
              // height
              min_proj = std::numeric_limits<float>::max();
              max_proj = std::numeric_limits<float>::lowest();
                  for (const auto& pt : cluster_pointcloud->points) {
                  Eigen::Vector3f p(pt.x, pt.y, pt.z);
                  float proj = shortest_eVec.dot(p - center); // project onto major axis
                  min_proj = std::min(min_proj, proj);
                  max_proj = std::max(max_proj, proj);
              }
              chili_height = max_proj - min_proj;
              std::cout << "Estimated chili height: " << chili_height << std::endl;
          } else {
              center = Eigen::Vector3f(cluster_pointcloud->points[0].x, cluster_pointcloud->points[0].y, cluster_pointcloud->points[0].z);

          }



          float chili_cross = chili_width/chili_height;
          std::cout << "Estimated chili cross section ratio: " << chili_cross << std::endl;



          // volume
          float l = chili_length / 2.0f;
          float w = chili_width / 2.0f;
          float h = chili_height / 2.0f;

          float chili_volume = (4.0f / 3.0f) * M_PI * l * w * h;
          std::cout << "Estimated chili volume: " << chili_volume << std::endl;



          // normalize
          float chili_length_norm = normalizeWithRange(chili_length, ideal_length);
          float chili_cross_norm = normalizeWithRange(chili_cross, ideal_cross);
          float chili_volume_norm = normalizeWithRange(chili_volume, ideal_volume);

          float d_score = std::sqrt(
              std::pow(w_length * chili_length_norm, 2) +
              std::pow(w_cross  * chili_cross_norm, 2) +
              std::pow(w_volume * chili_volume_norm, 2)
          );

          std::cout << "Normalized chili length: " << chili_length_norm<< std::endl;
          std::cout << "Normalized chili cross: " << chili_cross_norm << std::endl;
          std::cout << "Normalized chili volume: " << chili_volume_norm << std::endl;
          std::cout << "D score: " << d_score << std::endl;
          std::cout << "" << std::endl;


          chili_features[pair.first] = {d_score, center[0], center[1], center[2], chili_length_norm, chili_cross_norm, chili_volume_norm, chili_length};


      }

      std::vector<uint32_t> tagged;
      std::vector<Edit> planned_edits;
      std::vector<Edit> merge_edits;

      delete_edits.clear();


      // MERGE / SPLIT / DELETE
      for (const auto& pair : global_map) {
        auto cluster_id = pair.first;
        auto neighbor = getNearbySmallCluster(chili_features, cluster_id, merge_dist_threshold, tagged);
        if (chili_features.at(cluster_id)[0] > 0.5 && std::find(tagged.begin(), tagged.end(), cluster_id) == tagged.end()) {
            // big volume cluster -> split
            if (chili_features.at(cluster_id)[6] > 0.5f) {
                planned_edits.push_back({Edit::Split, cluster_id, 0});
                tagged.push_back(cluster_id);
            }
            // small volume cluster && nearby small volume clusters -> merge
            else if (neighbor != cluster_id) {
                merge_edits.push_back({Edit::Merge, cluster_id, neighbor});
                tagged.push_back(cluster_id);
                tagged.push_back(neighbor);
            }
            // lone small volume cluster
            else {
                delete_edits.push_back({Edit::Delete, cluster_id, 0});
                tagged.push_back(cluster_id);

            }
        }
      }


      // params for 2nd run
        float color_importance2 = config["counting"]["lccp2"]["color"];
        float spatial_importance2 = config["counting"]["lccp2"]["space"];
        float normal_importance2 = config["counting"]["lccp2"]["normal"];
        float concavity_tolerance_threshold2 = config["counting"]["lccp2"]["ct"];
        float smoothness_threshold2 = config["counting"]["lccp2"]["st"];
        //float voxel_resolution2 = 0.001;
        //float seed_resolution2 = 0.002;


        uint32_t max_label = 0;




        for (const auto& edit : planned_edits) {

          std::cout << edit.cluster_id << std::endl;

          auto cloud = global_map[edit.cluster_id];
          Eigen::Vector4f min_pt, max_pt;
          pcl::getMinMax3D(*cloud, min_pt, max_pt);
          
          
          //float dx = max_pt.x() - min_pt.x();
          //float dy = max_pt.y() - min_pt.y();
          //float dz = max_pt.z() - min_pt.z();
          //float max_dim = std::max({dx, dy, dz});  // C++11+
          // Choose voxel/seed relative to size
          float voxel_resolution2 = chili_features.at(edit.cluster_id)[7] / 10.0f;
          float seed_resolution2  = chili_features.at(edit.cluster_id)[7] / 5.0f;
          
          // Make sure they are not too small
          voxel_resolution2 = std::max(voxel_resolution2, 0.0001f);
          seed_resolution2  = std::max(seed_resolution2, 0.002f);
          
          pcl::SupervoxelClustering<PointT> super2 (voxel_resolution2, seed_resolution2);
          
          
          
          
                  // we do lccp again 
                  std::cout << "splitting" << std::endl;


                  // new supervoxel 
                  supervoxel_clusters.clear();
                  supervoxel_adjacency.clear();
                  //segment_map.clear();

                  super2.setInputCloud (global_map[edit.cluster_id]);
                  super2.setColorImportance (color_importance2);
                  super2.setSpatialImportance (spatial_importance2);
                  super2.setNormalImportance (normal_importance2);
                  pcl::console::print_highlight ("Extracting supervoxels!\n");
                  super2.extract (supervoxel_clusters);
                  pcl::console::print_info ("Found %d supervoxels\n", supervoxel_clusters.size ());
                  pcl::console::print_highlight ("Getting supervoxel adjacency\n");
                  super2.getSupervoxelAdjacency (supervoxel_adjacency);

                  // new lccp 
                  PCL_INFO("Starting Segmentation\n");
                  lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold2);
                  lccp.setSmoothnessCheck(true, voxel_resolution2, seed_resolution2, smoothness_threshold2);
                  lccp.setKFactor(k_factor);
                  lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
                  lccp.setMinSegmentSize(min_segment_size);
                  lccp.segment();
                  
                  PCL_INFO("Interpolation voxel cloud -> input cloud and relabeling\n");
                
                  sv_labeled_cloud = super2.getLabeledCloud();
                  lccp_labeled_cloud = sv_labeled_cloud->makeShared();
                  lccp.relabelCloud(*lccp_labeled_cloud);
                  //pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList sv_adjacency_list;
                  //lccp.getSVAdjacencyList(sv_adjacency_list);
                  //lccp.getSegmentToSupervoxelMap(segment_map);
                  //cout << "map size: " << segment_map.size() << endl; 

                  // add new splits into global map
                  if (!global_map.empty()) {
                      max_label = global_map.rbegin()->first;  // maps are ordered by key
                  }

                  for (const auto& pt : lccp_labeled_cloud->points)
                  {
                      // if label is invalid, skip
                      if (pt.label == 0)
                            continue;

                      // ensures new segments are given new labels
                      uint32_t label = pt.label + max_label;

                      // allocate cloud if first time seeing this label
                      if (!global_map.count(label))
                      {
                          global_map[label] = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
                      }

                      global_map[label]->points.emplace_back(pt.x, pt.y, pt.z);
                  }

                  //std::cout << "Number of segments: " << global_map.size() << std::endl;


                  /*for (const auto& pair : segment_map) {
                      
                      uint32_t label = ++max_label; 

                      for (const auto& sv_id : pair.second) {
                          auto sv = supervoxel_clusters.at(sv_id);  // lookup supervoxel by ID

                          // allocate cloud if first time seeing this label
                          if (!global_map[label]) {   // shared_ptr is null
                              PointCloudT::Ptr blank_cloud (new PointCloudT);
                              global_map[label] = blank_cloud;

                          }

                          // add all points from this supervoxel to the label’s cloud
                          *(global_map[label]) += *(sv->voxels_);
                      }
                  }*/

                  std::cout << "splits added"  << std::endl;

                  // delete the old one
                  global_map.erase(edit.cluster_id);
                  std::cout << "old splitter erased" << std::endl;

      }
      planned_edits.clear();



      for (const auto& edit : merge_edits) {
        std::cout << "merging" << std::endl;
        *(global_map[edit.cluster_id]) += *(global_map[edit.other_id]);
        global_map.erase(edit.other_id);
      }
      merge_edits.clear();

      std::cout << "\nNumber of clusters: " << global_map.size() << std::endl;

      

  }
      


      for (const auto& edit : delete_edits) {
        std::cout << "deleting" << std::endl;
        std::cout << "length: " << chili_features.at(edit.cluster_id)[4] << std::endl;
        std::cout << "cross: " << chili_features.at(edit.cluster_id)[5] << std::endl;
        std::cout << "volume: " << chili_features.at(edit.cluster_id)[6] << std::endl;
        std::cout << "dscore: " << chili_features.at(edit.cluster_id)[0] << std::endl;

        printf("\n");

        global_map.erase(edit.cluster_id); 
      }
      
      std::cout << "Number of clusters: " << global_map.size() << std::endl;



  // VISUALIZER
  /*
  pcl::visualization::PCLVisualizer::Ptr viewer3 (new pcl::visualization::PCLVisualizer ("3D Viewer"));
      viewer3->addPointCloud (lccp_labeled_cloud, "post global feature segmentation");
      while (!viewer3->wasStopped ())

    {
      viewer3->spinOnce (100);

    }
  */



  // Combine all clusters into one cloud (optional: color them differently)
  pcl::PointCloud<PointT>::Ptr combined(new pcl::PointCloud<PointT>);

  int i = 1;
  for (const auto& pair : global_map) {
      PointCloudT::Ptr cluster = pair.second;
      printf("size: %ld\n", cluster->size());

      for (auto& pt : cluster->points) {
          pt.r = (i * 53) % 256;
          pt.g = (i * 97) % 256;
          pt.b = (i * 193) % 256;
          pt.a = 1;
      }

      *combined += *cluster;
      i++;
  }

  // Visualize
  pcl::visualization::PCLVisualizer::Ptr viewer3(new pcl::visualization::PCLVisualizer("Global Map"));
  viewer3->addPointCloud<PointT>(combined, "global_cloud");
  viewer3->setBackgroundColor(0,0,0);

  while (!viewer3->wasStopped()) {
      viewer3->spinOnce(100);
  }

  // write supervoxel
  std::string final_name = basename + "_final.pcd";
  writer.write<PointT> (final_name, *combined, false);

  return (0);

}








inline void hsv2rgb(float h, float s, float v, uint8_t &r, uint8_t &g, uint8_t &b) {
    float c = v * s;
    float x = c * (1 - fabs(fmod(h / 60.0f, 2) - 1));
    float m = v - c;

    float r1, g1, b1;
    if (h < 60)      { r1 = c; g1 = x; b1 = 0; }
    else if (h < 120){ r1 = x; g1 = c; b1 = 0; }
    else if (h < 180){ r1 = 0; g1 = c; b1 = x; }
    else if (h < 240){ r1 = 0; g1 = x; b1 = c; }
    else if (h < 300){ r1 = x; g1 = 0; b1 = c; }
    else             { r1 = c; g1 = 0; b1 = x; }

    r = static_cast<uint8_t>((r1 + m) * 255);
    g = static_cast<uint8_t>((g1 + m) * 255);
    b = static_cast<uint8_t>((b1 + m) * 255);
}






uint32_t getNearbySmallCluster(const std::map<std::uint32_t, std::vector<float>>& c_features, uint32_t id, float dist_threshold, std::vector<uint32_t> tagged) {

  Eigen::Vector3f self_center(c_features.at(id)[1], c_features.at(id)[2], c_features.at(id)[3]);

  for (const auto& pair : c_features) {
    Eigen::Vector3f other_center(pair.second[1], pair.second[2], pair.second[3]);
    if (std::find(tagged.begin(), tagged.end(), pair.first) != tagged.end())
      continue ;
    if (pair.first == id)
      continue ;
    if ((self_center - other_center).norm() < dist_threshold) 
      return pair.first;
  }

  return id;


}


float normalizeWithRange(float value, std::pair<float,float> ideal_range) {
if (value < ideal_range.first)
  return (value - ideal_range.first) / (ideal_range.second - ideal_range.first);
else if (value > ideal_range.second)
  return (value - ideal_range.second) / (ideal_range.second - ideal_range.first);
else
  return 0;

}

void

addSupervoxelConnectionsToViewer (PointT &supervoxel_center,

                                  PointCloudT &adjacent_supervoxel_centers,

                                  std::string supervoxel_name,

                                  pcl::visualization::PCLVisualizer::Ptr & viewer)

{

  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();

  vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();

  vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();


  //Iterate through all adjacent points, and add a center point to adjacent point pair

  for (auto adjacent_itr = adjacent_supervoxel_centers.begin (); adjacent_itr != adjacent_supervoxel_centers.end (); ++adjacent_itr)

  {

    points->InsertNextPoint (supervoxel_center.data);

    points->InsertNextPoint (adjacent_itr->data);

  }

  // Create a polydata to store everything in

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();

  // Add the points to the dataset

  polyData->SetPoints (points);

  polyLine->GetPointIds  ()->SetNumberOfIds(points->GetNumberOfPoints ());

  for(unsigned int i = 0; i < points->GetNumberOfPoints (); i++)

    polyLine->GetPointIds ()->SetId (i,i);

  cells->InsertNextCell (polyLine);

  // Add the lines to the dataset

  polyData->SetLines (cells);

  viewer->addModelFromPolyData (polyData,supervoxel_name);

}