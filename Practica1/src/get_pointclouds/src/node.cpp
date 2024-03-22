#include "common.h"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr last_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr last_keypoints(new pcl::PointCloud<pcl::PointXYZRGB>());
pcl::PointCloud<pcl::Normal>::Ptr last_normals(new pcl::PointCloud<pcl::Normal>());
pcl::PointCloud<pcl::FPFHSignature33>::Ptr last_descriptors(new pcl::PointCloud<pcl::FPFHSignature33>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
pcl::CorrespondencesPtr estimateCorrespondences(new pcl::Correspondences);
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> transformed_clouds;


Eigen::Matrix4f transformation;
double resolution;

double get_cloud_resolution(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud){
	double res = 0.0;
  	int n_points = 0, n_res;
  	std::vector<int> indices (2);
  	std::vector<float> sqr_distances (2);
  	pcl::search::KdTree<pcl::PointXYZRGB> tree;
  	tree.setInputCloud(cloud); 
	for(size_t i=0;i<cloud->size();++i) {
		if(!std::isfinite((*cloud)[i].x)) 
			continue;
		n_res = tree.nearestKSearch(i, 2, indices, sqr_distances); 
		if (n_res == 2) {
      		res += sqrt(sqr_distances[1]);
      		++n_points;
    	} 
	}
	if(n_points != 0)
		res /= n_points;
	return res;
}

void remove_nan(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>());
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *output, indices);
	*cloud = *output;
	std::cout << "Number of points after remove_nan: " << cloud->size() << "\n";
}

void estimate_normals(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud, pcl::PointCloud<pcl::Normal>::Ptr normals){
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	normalEstimation.setRadiusSearch(0.03);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);
    std::cout << "Number of normals estimated: " << normals->size() << "\n";
}

void iss_keypoints(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& keypoints) {
    pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss_detector;

    iss_detector.setSalientRadius(7 * resolution);
    iss_detector.setNonMaxRadius(3 * resolution);
    iss_detector.setThreshold21(0.7);
    iss_detector.setThreshold32(0.4);
    iss_detector.setMinNeighbors(3);
    iss_detector.setNumberOfThreads(4);
    iss_detector.setInputCloud(cloud);

    iss_detector.compute(*keypoints);

    // Colorear keypoints en verde
    for(size_t i = 0; i < keypoints->size(); ++i){
        keypoints->points[i].r = 0;
        keypoints->points[i].g = 255;
        keypoints->points[i].b = 0;
    }

    std::cout << "Numero de keypoints: " << keypoints->size() << "\n";
}

void FPFH_descriptors(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &keypoints, pcl::PointCloud<pcl::FPFHSignature33>::Ptr &descriptors){
	pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>());

	estimate_normals(keypoints, normals);
	fpfh.setInputCloud(keypoints);
	fpfh.setInputNormals(normals);
	fpfh.setSearchMethod(tree);
	fpfh.setRadiusSearch(0.1);
	fpfh.compute(*descriptors);

	cout << "Number of descriptors with FPFH: " << descriptors->size() << "\n";	
}

void matching(pcl::PointCloud<pcl::FPFHSignature33>::Ptr &descriptors){
	pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> corr;
	corr.setInputSource(descriptors);
	corr.setInputTarget(last_descriptors);
	corr.determineCorrespondences(*estimateCorrespondences);
	std::cout << "Numero de correspondencias encontradas: " << estimateCorrespondences->size() << std::endl;
}

Eigen::Matrix4f ransac(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &keypoints, pcl::CorrespondencesPtr bestCorrespondences){
	//RANSAC
	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB> crsc;
    crsc.setInputSource(keypoints);
    crsc.setInputTarget(last_keypoints);
    crsc.setInlierThreshold(0.25);
    crsc.setMaximumIterations(1000);
    crsc.setRefineModel(true);
    crsc.setInputCorrespondences(estimateCorrespondences);
	crsc.getCorrespondences(*bestCorrespondences);
	return crsc.getBestTransformation();
}

void iterative_closest_point(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud){
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>());
    
    icp.setInputSource(cloud_transformed);
    icp.setInputTarget(last_filtered);
    icp.setMaxCorrespondenceDistance(0.1);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1);
    pcl::PointCloud<pcl::PointXYZRGB> aligned_cloud;
    icp.align(aligned_cloud);

    if(icp.hasConverged()){
        transformation = icp.getFinalTransformation();
    }

	std::cout << "ICP Score: " << icp.getFitnessScore() << "\n";
	std::cout << "ICP matrix transformation: \n" << transformation << "\n";
}

void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(*msg));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors(new pcl::PointCloud<pcl::FPFHSignature33>());

	cout << "Puntos capturados: " << cloud->size() << endl;

	remove_nan(cloud);

	cout << "Puntos filtrados: " << cloud->size() << endl;

	//VoxelGrid
	pcl::VoxelGrid<pcl::PointXYZRGB > vGrid;
	vGrid.setInputCloud(cloud);
	vGrid.setLeafSize (0.04f, 0.04f, 0.04f);
	vGrid.filter(*cloud_filtered);

	cout << "Puntos tras VG: " << cloud_filtered->size() << endl;

	//Normales
	resolution = get_cloud_resolution(cloud_filtered);
	estimate_normals(cloud_filtered, normals);

	//Keypoints
	iss_keypoints(cloud_filtered, keypoints);
	*cloud_filtered += *keypoints;

	//Descriptores
	FPFH_descriptors(keypoints, descriptors);

	//Matching
	if(!last_filtered->empty()){ //no podemos hacer matching con solo 1 nube, necesitamos 2
		matching(descriptors);

		pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
		transformation = ransac(keypoints, correspondences);
		//ICP
		//iterative_closest_point(cloud_filtered);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::transformPointCloud(*cloud_filtered, *transformed_cloud, transformation);
		std::cout << "Transformation matrix: \n" << transformation << "\n";

		*final_cloud += *transformed_cloud;
	}	

	*last_filtered  = *cloud_filtered;
	*last_keypoints = *keypoints;
	*last_descriptors = *descriptors;
	*last_normals = *normals;
}

void simpleVis(){
  	pcl::visualization::CloudViewer viewer ("Cloud Viewer");
	while(!viewer.wasStopped()){
	  viewer.showCloud(final_cloud);
	  boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}
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