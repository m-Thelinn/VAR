#include "node.h"

std::vector<pcl::PointCloud<PointType>::Ptr> nubes;

pcl::PointCloud<PointType>::Ptr visu_pc (new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr last (new pcl::PointCloud<PointType>());


void eliminarNaN(pcl::PointCloud<PointType>::Ptr cloud){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>());
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *output, indices);
	*cloud = *output;
}


void VoxelGrid_Filter(pcl::PointCloud<PointType>::Ptr& c, pcl::PointCloud<PointType>::Ptr& c_f)
{
	const static float l_size = 0.02f;

	pcl::VoxelGrid<PointType> vGrid;
	vGrid.setInputCloud(c);
	vGrid.setLeafSize(l_size, l_size, l_size);
	vGrid.filter(*c_f);

	cout << "Puntos VoxelGrid: " << c_f->size() << "\033[32m" << endl;
}


void StatisticalOutlier_Filter(pcl::PointCloud<PointType>::Ptr& c, pcl::PointCloud<PointType>::Ptr& c_f)
{
	const static float multhresh = 0.5f;
	const static int mean = 100;

	pcl::StatisticalOutlierRemoval<PointType> sor;
	sor.setInputCloud(c);
	sor.setMeanK(mean);
	sor.setStddevMulThresh(multhresh);
	sor.filter(*c_f);

	cout << "Puntos SOR: " << c_f->size() << "\033[34m" << endl;
}



void Calculate_Normals(pcl::PointCloud<PointType>::Ptr& c, pcl::PointCloud<pcl::PointNormal>::Ptr& c_n)
{
	const static float radius = 0.045f;

	pcl::NormalEstimation<PointType, pcl::PointNormal> normals;
	pcl::search::KdTree<PointType>::Ptr ktree(new pcl::search::KdTree<PointType>());

	normals.setInputCloud(c);
	normals.setSearchMethod(ktree);
	normals.setRadiusSearch(radius);
	normals.setSearchSurface(c);
	normals.compute(*c_n);
}



void KeypointManager(pcl::PointCloud<PointType>::Ptr& c, pcl::PointCloud<pcl::PointNormal>::Ptr& c_n, pcl::PointCloud<pcl::PointWithScale>::Ptr& keys)
{

	// SIFT
	#if Keypoint	== 1

		const static float min_scale = 0.01f;
		const static int n_octaves = 3;
		const static int n_scales_per_octave = 4;
		const static float min_contrast = 0.001f;

		pcl::SIFTKeypoint<PointType, pcl::PointWithScale> sift;
		pcl::search::KdTree<PointType>::Ptr ktree(new pcl::search::KdTree<PointType>());

		sift.setSearchMethod(ktree);
		sift.setScales(min_scale, n_octaves, n_scales_per_octave);
		sift.setMinimumContrast(min_contrast);
		sift.setInputCloud(c);
		sift.compute(*keys);

		cout << "Keypoints SIFT: " << keys->size() << "\033[35m" << endl;


	// ISS
	#elif Keypoint	== 2

		const static int salient_radius = 5;
		const static int max_radius = 3;

		pcl::ISSKeypoint3D<PointType, PointType> iss;
		pcl::search::KdTree<PointType>::Ptr ktree(new pcl::search::KdTree<PointType>());
		
		iss.setNonMaxRadius(max_radius);
		iss.setSalientRadius(salient_radius);
		iss.setInputCloud(c);
		iss.compute(*keys);

		cout << "Keypoints ISS: " << keys->size() << endl;


	// HARRIS-3D
	#elif Keypoint	== 3

		pcl::HarrisKeypoint3D<PointType, pcl::PointXYZI> harris;
		harris.setNonMaxSupression(true);
		harris.setThreshold(1e-9);
		harris.setInputCloud(c_n);
		harris.compute(*keys);

		cout << "Keypoints HARRIS-3D: " << keys->size() << endl;

	#endif

}


void DescriptorManager(pcl::PointCloud<PointType>::Ptr& c, pcl::PointCloud<pcl::PointNormal>::Ptr& c_n, 
pcl::PointCloud<pcl::PointWithScale>::Ptr& keys, pcl::PointCloud<DescriptorType>::Ptr& fpfh)
{
	pcl::search::KdTree<PointType>::Ptr ktree(new pcl::search::KdTree<PointType>());
	const static float radius = 0.05f;

	pcl::PointCloud<PointType>::Ptr keypoints_xyzrgb(new pcl::PointCloud<PointType>());
	pcl::copyPointCloud(*keys, *keypoints_xyzrgb);
	

	// FPFH
	#if Descriptor	== 1
		pcl::FPFHEstimation<PointType, pcl::PointNormal, DescriptorType> fpfh_d;

		fpfh_d.setInputCloud(keypoints_xyzrgb);
		fpfh_d.setInputNormals(c_n);
		fpfh_d.setSearchSurface(c);

		fpfh_d.setSearchMethod(ktree);
		fpfh_d.setRadiusSearch(radius);

		fpfh_d.compute(*fpfh);

		cout << "Descriptores FPFH: " << fpfh->size() << "\033[33m" << endl;
	
	// CVFH & VFH
	#elif Descriptor == 2
		pcl::CVFHEstimation<PointType, pcl::PointNormal, DescriptorType> vfh;
		vfh.setInputCloud(c);
		vfh.setInputNormals(c_n);
		vfh.setSearchMethod(ktree);
		//vfh.setNormalizeBins(true); //VFH
		//vfh.setNormalizeDistance(false); //VFH

		//CVFH
		vfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI);
		vfh.setCurvatureThreshold(1.0);
	
		vfh.setNormalizeBins(false); //CVFH
		vfh.compute(*fpfh);

		cout << "Descriptores VFH: " << fpfh->size() << "\033[33m" << endl;

	// SHOT352
	#elif Descriptor == 3
		pcl::SHOTEstimationOMP<PointType, pcl::PointNormal, DescriptorType> shot;
		shot.setRadiusSearch(radius);
		shot.setInputCloud(keypoints_xyzrgb);
		shot.setInputNormals(c_n);
		shot.setSearchSurface(c);
		shot.compute(*fpfh);
		
	#endif
}


Eigen::Matrix4f RANSAC2(pcl::PointCloud<PointType>::Ptr& k1, pcl::PointCloud<DescriptorType>::Ptr& d1, pcl::PointCloud<PointType>::Ptr& k2, pcl::PointCloud<DescriptorType>::Ptr& d2)
{
	pcl::SampleConsensusInitialAlignment<PointType, PointType, DescriptorType> sac_ia;

	const float min_sample_dist = 0.025f;
	const float max_correspondence_dist = 0.01f;
	const int nr_iters = 500;

	sac_ia.setMinSampleDistance(min_sample_dist);
	sac_ia.setMaxCorrespondenceDistance(max_correspondence_dist);
	sac_ia.setMaximumIterations(nr_iters);

	sac_ia.setInputCloud(k1);
	sac_ia.setSourceFeatures(d1);

	sac_ia.setInputTarget(k2);
	sac_ia.setTargetFeatures(d2);

	pcl::PointCloud<PointType> alignment;
	sac_ia.align(alignment);

	return (sac_ia.getFinalTransformation ());
}



void Matching(pcl::PointCloud<DescriptorType>::Ptr& prev, pcl::PointCloud<DescriptorType>::Ptr& next, pcl::Correspondences& correspondences)
{
	pcl::registration::CorrespondenceEstimation<DescriptorType, DescriptorType> estimacion;
	estimacion.setInputSource(next);
	estimacion.setInputTarget(prev);

	estimacion.determineReciprocalCorrespondences(correspondences);

	// ToDo Rejected
}



Eigen::Matrix4f Transformation(pcl::PointCloud<pcl::PointXYZ>::Ptr& keyInput, pcl::Correspondences& corrInput,
							   pcl::PointCloud<pcl::PointXYZ>::Ptr& keyTarget, pcl::Correspondences& corrTarget)
{
	const static int max_iter = 1500;
	const static float threshold = 0.5f;

	pcl::CorrespondencesConstPtr estimacion(new pcl::Correspondences(corrInput));
	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> crsc;

	crsc.setInputSource(keyTarget);
	crsc.setInputTarget(keyInput);

	crsc.setRefineModel(true);
	crsc.setInlierThreshold(threshold);
	crsc.setMaximumIterations(max_iter);
	crsc.setInputCorrespondences(estimacion);
	crsc.getCorrespondences(corrTarget);

	//cout << crsc.getBestTransformation() << endl;	
	//return crsc.getBestTransformation();


	Eigen::Matrix4f transform; 
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> transformSVD;

	transformSVD.estimateRigidTransformation (*keyTarget, *keyInput, corrTarget, transform); 

	return transform;
}



void RANSAC(pcl::PointCloud<PointType>::Ptr& cloud, pcl::CorrespondencesPtr corres, Eigen::Matrix4f& trans)
{
	pcl::CorrespondencesPtr corr(new pcl::Correspondences);
	
	pcl::registration::CorrespondenceEstimation<PointType, PointType> corr_est;
	corr_est.setInputSource(last);
	corr_est.setInputTarget(cloud);
	corr_est.determineCorrespondences(*corr);

	// -----------------------

	const static int max_iter = 8000;
	const static float threshold = 0.025f;
	
	pcl::registration::CorrespondenceRejectorSampleConsensus<PointType> crsc;
	crsc.setInputSource(last);
	crsc.setInputTarget(cloud);
	crsc.setRefineModel(true);
	crsc.setInlierThreshold(threshold);
	crsc.setMaximumIterations(max_iter);
	crsc.setInputCorrespondences(corr);
	crsc.getCorrespondences(*corres);

	trans = crsc.getBestTransformation();
	cout << "Est. correspondences: " << corr->size() << endl;
	cout << "Remaining correspondences: " << corres->size() << endl; 
	//cout << trans << endl;	
}



void ICP(pcl::PointCloud<PointType>::Ptr& cloud, Eigen::Matrix4f& trans)
{
	pcl::IterativeClosestPoint<PointType, PointType> icp;

	icp.setInputCloud(last);
	icp.setInputTarget(cloud);

	icp.setMaxCorrespondenceDistance(0.35); // 20 cm?
	icp.setMaximumIterations(40);
	icp.setTransformationEpsilon(1e-8);
	icp.setEuclideanFitnessEpsilon(1);

	
	pcl::PointCloud<PointType>::Ptr alignment(new pcl::PointCloud<PointType>);
	icp.align(*alignment, trans);
	nubes.push_back(alignment); 

	if(icp.hasConverged()) { trans = icp.getFinalTransformation(); }
}




// ==========================================



void simpleVis ()
{
  	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	while(!viewer.wasStopped())
	{
	  viewer.showCloud (visu_pc);
	  boost::this_thread::sleep(boost::posix_time::milliseconds(800));
	}

}


void Mapping(const pcl::PointCloud<PointType>::Ptr& c)
{
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
	pcl::copyPointCloud(*c, *cloud);

	// NORMALS ================================================================================

		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_1(new pcl::PointCloud<pcl::PointNormal>());
		Calculate_Normals(last, cloud_normals_1);

		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_2(new pcl::PointCloud<pcl::PointNormal>());
		Calculate_Normals(cloud, cloud_normals_2);

	// ========================================================================================

	// KEYPOINTS ==============================================================================

		pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_1(new pcl::PointCloud<pcl::PointWithScale>());
		KeypointManager(last, cloud_normals_1, keypoints_1);

		pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_2(new pcl::PointCloud<pcl::PointWithScale>());
		KeypointManager(cloud, cloud_normals_2, keypoints_2);

		pcl::PointCloud<PointType>::Ptr keypoints_xyzrgb_1(new pcl::PointCloud<PointType>());
		pcl::copyPointCloud(*keypoints_1, *keypoints_xyzrgb_1);
		pcl::PointCloud<PointType>::Ptr keypoints_xyzrgb_2(new pcl::PointCloud<PointType>());
		pcl::copyPointCloud(*keypoints_2, *keypoints_xyzrgb_2);
		//*visu_pc +=  *keypoints_xyzrgb; // mostrar keypoints en visualizacion


		pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_target_1(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::copyPointCloud(*keypoints_1, *keypoints_target_1);

		pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_target_2(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::copyPointCloud(*keypoints_2, *keypoints_target_2);

	// ========================================================================================

	cout << "\n\n";

	// DESCRIPTOR =============================================================================

		pcl::PointCloud<DescriptorType>::Ptr fpfh_descriptor_1(new pcl::PointCloud<DescriptorType>());
		DescriptorManager(last, cloud_normals_1, keypoints_1, fpfh_descriptor_1);

		pcl::PointCloud<DescriptorType>::Ptr fpfh_descriptor_2(new pcl::PointCloud<DescriptorType>());
		DescriptorManager(cloud, cloud_normals_2, keypoints_2, fpfh_descriptor_2);

	// ========================================================================================

	cout << "\n\n";


	// MATCHING ===============================================================================

		Eigen::Matrix4f transformation;
		//transformation = RANSAC2(keypoints_xyzrgb_1, fpfh_descriptor_1, keypoints_xyzrgb_2, fpfh_descriptor_2);
		
		/*pcl::CorrespondencesPtr correspondence (new pcl::Correspondences());
		RANSAC(cloud, correspondence, transformation);*/
		
		pcl::Correspondences correspondences;
		Matching(fpfh_descriptor_1, fpfh_descriptor_2, correspondences); // da matriz de identidad xq?

		pcl::Correspondences rejected;
		transformation = Transformation(keypoints_target_1, correspondences, keypoints_target_2, rejected);

		cout << transformation << endl;

	// ========================================================================================

	// ICP ====================================================================================
		/*Eigen::Matrix4f transformation2;
		ICP(cloud, transformation2);
		cout << transformation2 << endl;
		transformation *= transformation2;*/
	// ========================================================================================

	pcl::PointCloud<PointType>::Ptr cloud_new(new pcl::PointCloud<PointType>());
	pcl::transformPointCloud(*cloud, *cloud_new, transformation);

	/*pcl::PointCloud<PointType>::Ptr cloud_new_filtered(new pcl::PointCloud<PointType>());
	VoxelGrid_Filter(cloud_new, cloud_new_filtered);*/

	*last = *cloud_new;
	*visu_pc += *cloud_new;
}



void Filtrado(const pcl::PointCloud<PointType>::Ptr& c)
{
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
	pcl::copyPointCloud(*c, *cloud);

	eliminarNaN(cloud);
	
	pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>());
	VoxelGrid_Filter(cloud, cloud_filtered);

	pcl::PointCloud<PointType>::Ptr cloud_filtered_sor(new pcl::PointCloud<PointType>());
	StatisticalOutlier_Filter(cloud_filtered, cloud_filtered_sor);

	cout << "\n\n";

	//nubes.push_back(cloud_filtered_sor); //pcds

	*cloud = *cloud_filtered_sor;
	//*cloud = *cloud_filtered;
	pcl::copyPointCloud(*cloud, *c);
}



void callback(const pcl::PointCloud<PointType>::ConstPtr& msg)
{
	pcl::PointCloud<PointType>::Ptr cloud_OG(new pcl::PointCloud<PointType>(*msg)); // OG

	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>(*msg));
	cout << "Puntos capturados: " << cloud->size() << "\033[31m" << endl;

	Filtrado(cloud);

	if(!last->empty())	   { Mapping(cloud); }
	else { last = cloud; *visu_pc += *cloud; }
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<pcl::PointCloud<PointType> >("/camera/depth/points", 1, callback);

  boost::thread t(simpleVis);

  while(ros::ok())
  {
	ros::spinOnce();
  }
}