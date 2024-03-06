#include "common.h"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr visu_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZRGB>);

double resolution;

void simpleVis(){
  	pcl::visualization::CloudViewer viewer ("Cloud Viewer");
	while(!viewer.wasStopped()){
	  viewer.showCloud (visu_pc);
	  boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}
}

double get_cloud_resolution(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
{
	double res = 0.0;
  	int n_points = 0, n_res;
  	std::vector<int> indices (2);
  	std::vector<float> sqr_distances (2);
  	pcl::search::KdTree<pcl::PointXYZRGB> tree;
  	tree.setInputCloud(cloud); 
	for(size_t i=0;i<cloud->size();++i) {
		if(!std::isfinite((*cloud)[i].x)) 
			continue;
		n_res = tree.nearestKSearch (i, 2, indices, sqr_distances); 
		if (n_res == 2) {
      		res += sqrt(sqr_distances[1]);
      		++n_points;
    	} 
	}
	if(n_points != 0)
		res /= n_points;
	return res;
}

void iss_keypoints(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& keypoints) {
    pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss_detector;

    iss_detector.setSalientRadius(6 * resolution);
    iss_detector.setNonMaxRadius(4 * resolution);
    iss_detector.setThreshold21(0.975);
    iss_detector.setThreshold32(0.975);
    iss_detector.setMinNeighbors(5);
    iss_detector.setNumberOfThreads(4);
    iss_detector.setInputCloud(cloud);

    iss_detector.compute(*keypoints);

    // Colorear keypoints en verde
    for (size_t i = 0; i < keypoints->size(); ++i) {
        keypoints->points[i].r = 0;
        keypoints->points[i].g = 255;
        keypoints->points[i].b = 0;
    }

    std::cout << "Numero de keypoints: " << keypoints->size() << "\n";
    // std::cout << "ISS Time: " << get_cpu_time() - before_time << "sec\n";
}


void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>(*msg));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
	cout << "Puntos capturados: " << cloud->size() << endl;

	pcl::VoxelGrid<pcl::PointXYZRGB > vGrid;
	vGrid.setInputCloud (cloud);
	vGrid.setLeafSize (0.04f, 0.04f, 0.04f);
	vGrid.filter (*cloud_filtered);

	cout << "Puntos tras VG: " << cloud_filtered->size() << endl;

	//Keypoints
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	resolution = get_cloud_resolution(cloud_filtered);
	iss_keypoints(cloud_filtered, keypoints);
	*cloud_filtered += *keypoints;

	visu_pc = cloud_filtered;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/camera/depth/points", 1, callback);

  boost::thread t(simpleVis);

  while(ros::ok()){
	ros::spinOnce();
  }
}
