#include <iostream>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/common/common.h>

ros::Publisher pub;

void 
cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud_blob)
{
  pcl::PCLPointCloud2::Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Crear el objeto a filtrar, haciendo un downsample de la entrada usando un tamaño de hoja de 1 cm.
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.02f, 0.02f, 0.02f);
  // El resultado lo introducims en cloud_filtered_blob
  sor.filter (*cloud_filtered_blob);

  // Convertimos la nube binaria, a un formato mas manejable en cloud_filtered
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Creamos el plano de segmentación
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  //seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);
  seg.setInputCloud (cloud_filtered);
  
  // Calculamos el numero de puntos.  
  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  pcl::PointXYZRGB min_pt,max_pt;
  //pcl::getMinMax3D(&cloud_filtered,min_pt,max_pt);

  pcl::IndicesPtr remaining (new std::vector<int>);
  remaining->resize (nr_points);
  for (size_t i = 0; i < remaining->size (); ++i) { (*remaining)[i] = static_cast<int>(i); }

  // While 30% of the original cloud is still there
  while (remaining->size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setIndices (remaining);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0) break;

    // Extract the inliers
    std::vector<int>::iterator it = remaining->begin();
    for (size_t i = 0; i < inliers->indices.size (); ++i)
    {
      int curr = inliers->indices[i];
      // Remove it from further consideration.
      while (it != remaining->end() && *it < curr) { ++it; }
      if (it == remaining->end()) break;
      if (*it == curr) it = remaining->erase(it);
    }
    i++;
  }
  std::cout << "Encontrados " << i << " planos. Tenemos " << nr_points << std::endl;
  // Definimos el color verde
    uint8_t r = 0, g = 255, b = 0;
    uint8_t rojo_r = 255, rojo_g = 0, rojo_b = 0;
    uint32_t verde_rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    uint32_t rojo_rgb = ((uint32_t)rojo_r << 16 | (uint32_t)rojo_g << 8 | (uint32_t)rojo_b);
  // Color all the non-planar things.
  for (std::vector<int>::iterator it = remaining->begin(); it != remaining->end(); ++it)
  {
//std::cout << "x " << cloud_filtered->at(*it).x << "  y "<< cloud_filtered->at(*it).y << "  z " << cloud_filtered->at(*it).z << "  z" << std::endl;
if (cloud_filtered->at(*it).z>2)
{
    cloud_filtered->at(*it).rgb = *reinterpret_cast<float*>(&rojo_rgb);
}
else if(cloud_filtered->at(*it).y<0.2)
{
    cloud_filtered->at(*it).rgb = *reinterpret_cast<float*>(&rojo_rgb);
}
else
{
    cloud_filtered->at(*it).rgb = *reinterpret_cast<float*>(&verde_rgb);
}

  }

  // Publish the planes we found.
  pcl::PCLPointCloud2 outcloud;
  pcl::toPCLPointCloud2 (*cloud_filtered, outcloud);
  pub.publish (outcloud);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "planes");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PCLPointCloud2> ("planes", 1);

  // Spin
  ros::spin ();
}

