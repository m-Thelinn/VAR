#include <iostream>
#include <string>
#include <chrono>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/foreach.hpp>
#include <boost/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/shot_omp.h>
#include <pcl/correspondence.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr nube_filtrada_anterior(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr nube_key_anterior(new pcl::PointCloud<pcl::PointXYZRGB>());
//pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptores_anteriores(new pcl::PointCloud<pcl::PFHSignature125>());
pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptores_anteriores(new pcl::PointCloud<pcl::FPFHSignature33>());
//pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptores_anteriores(new pcl::PointCloud<pcl::VFHSignature308>());
//pcl::PointCloud<pcl::SHOT352>::Ptr descriptores_anteriores(new pcl::PointCloud<pcl::SHOT352>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr nube_visualizada(new pcl::PointCloud<pcl::PointXYZRGB>());
pcl::CorrespondencesPtr correspondenciaEstimada(new pcl::Correspondences);

Eigen::Matrix4f transformacion_actual;
Eigen::Matrix4f transformacion_global;
double resolucionNube;

void eliminarNaN(pcl::PointCloud<pcl::PointXYZRGB>::Ptr nube){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr nube_limpia(new pcl::PointCloud<pcl::PointXYZRGB>());
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*nube, *nube_limpia, indices);
	*nube = *nube_limpia;
}

double obtenerResolucion(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& nube_input){
	double res = 0.0;
  	int n_points = 0;
	int n_res;
  	std::vector<int> indices(2);
  	std::vector<float> sqr_distances(2);
  	pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
  	kdtree.setInputCloud(nube_input); 
	for(size_t i = 0;i < nube_input->size(); i++){
		if(!std::isfinite((*nube_input)[i].x)){
			continue;
		}
		n_res = kdtree.nearestKSearch(i, 2, indices, sqr_distances); 
		if(n_res == 2){
      		res += sqrt(sqr_distances[1]);
      		++n_points;
    	} 
	}
	if(n_points != 0){
		res /= n_points;
	}
	return res;
}

void obtenerNormales(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& nube_input, pcl::PointCloud<pcl::Normal>::Ptr normales){
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);

	ne.setInputCloud(nube_input);
	ne.setSearchMethod(kdtree);
	ne.setRadiusSearch(0.15);
	ne.setSearchSurface(nube_input);
	ne.compute(*normales);

    std::cout << "Numero de normales: " << normales->size() << "\n";
}

void ISS_keypoints(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& nube_input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& keypoints) {
    pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss;

	iss.setInputCloud(nube_input);
    iss.setSalientRadius(2.5 * resolucionNube);
    iss.setNonMaxRadius(2.5 * resolucionNube);
    iss.setThreshold21(0.95);
    iss.setThreshold32(0.95);
    iss.setMinNeighbors(4);
	iss.compute(*keypoints);

    for(size_t i = 0; i < keypoints->size(); ++i){ //pintar de verde
        keypoints->points[i].r = 0;
        keypoints->points[i].g = 255;
        keypoints->points[i].b = 0;
    }

    std::cout << "Numero de keypoints: " << keypoints->size() << "\n";
}

void SIFT_keypoints(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& nube_input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& keypoints){
	pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift;
	pcl::PointCloud<pcl::PointWithScale> resultado;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>());

	sift.setInputCloud(nube_input);

	sift.setSearchMethod(kdtree);
	sift.setScales(0.01, 3, 4);
	sift.setMinimumContrast(0.001f);
	sift.compute(resultado);
	copyPointCloud(resultado, *keypoints);

	for(size_t i = 0; i < keypoints->size(); ++i){
        keypoints->points[i].r = 0;
        keypoints->points[i].g = 255;
        keypoints->points[i].b = 0;
    }

	std::cout << "Numero de keypoints: " << keypoints->size() << "\n";
}

void Harris_keypoints(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& nube_input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& keypoints){
	pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI> harris;
	pcl::PointCloud<pcl::PointXYZI> resultado;

	harris.setNonMaxSupression(true);
	harris.setThreshold(1e-9);
	harris.setInputCloud(nube_input);
	harris.compute(resultado);
	copyPointCloud(resultado, *keypoints);

	for(size_t i = 0; i < keypoints->size(); ++i){
        keypoints->points[i].r = 0;
        keypoints->points[i].g = 255;
        keypoints->points[i].b = 0;
    }

	std::cout << "Numero de keypoints: " << keypoints->size() << "\n";
}

void PFH_descriptores(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &nube_input, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &keypoints, pcl::PointCloud<pcl::PFHSignature125>::Ptr &descriptores){
	pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfh;
	pcl::PointCloud<pcl::Normal>::Ptr normales(new pcl::PointCloud<pcl::Normal>());
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>());

	obtenerNormales(keypoints, normales);
	pfh.setInputCloud(keypoints);
	pfh.setInputNormals(normales);
	pfh.setSearchMethod(kdtree);
	pfh.setRadiusSearch(0.5);
	pfh.compute(*descriptores);

	cout << "Numero de descriptores PFH: " << descriptores->size() << "\n";	
}

void FPFH_descriptores(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &nube_input, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &keypoints, pcl::PointCloud<pcl::FPFHSignature33>::Ptr &descriptores){
	pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
	pcl::PointCloud<pcl::Normal>::Ptr normales(new pcl::PointCloud<pcl::Normal>());
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>());

	obtenerNormales(keypoints, normales);
	fpfh.setInputCloud(keypoints);
	fpfh.setInputNormals(normales);
	fpfh.setSearchMethod(kdtree);
	fpfh.setRadiusSearch(0.5);
	fpfh.compute(*descriptores);

	cout << "Numero de descriptores FPFH: " << descriptores->size() << "\n";	
}

//--------------------------------------------
//			NO FUNCIONA
//--------------------------------------------
//Se queda calculando infinitamente, sin importar que parametros le introduzcas
/*void CVFH_descriptores(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &nube_input, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& keypoints, pcl::PointCloud<pcl::VFHSignature308>::Ptr& descriptores){
	pcl::CVFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> cvfh;
	pcl::PointCloud<pcl::Normal>::Ptr normales(new pcl::PointCloud<pcl::Normal>());
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZRGB>());

	obtenerNormales(nube_input, normales);

	cvfh.setInputCloud(keypoints);
	cvfh.setInputNormals(normales);
	cvfh.setSearchMethod(kdtree);

	cvfh.setEPSAngleThreshold(0.0001);
	cvfh.setCurvatureThreshold(0.00001);
	cvfh.setNormalizeBins(false);
 
	cvfh.compute(*descriptores);	

	cout << "Numero de descriptores CVFH: " << descriptores->size() << "\n";	
}



//[pcl::SHOTEstimation::initCompute] The number of points in the input dataset (1450423) differs from the number of points in the dataset containing the normals (5406)!
//[pcl::SHOTEstimation::computeFeature] The local reference frame is not valid! Aborting description of point with index 3374
//get_pointclouds_node: /build/pcl-gWGA5r/pcl-1.10.0+dfsg/kdtree/include/pcl/kdtree/impl/kdtree_flann.hpp:138: int pcl::KdTreeFLANN<PointT, Dist>::nearestKSearch(const PointT&, int, std::vector<int>&, std::vector<float>&) const [with PointT = pcl::SHOT352; Dist = flann::L2_Simple<float>]: Assertion `point_representation_->isValid (point) && "Invalid (NaN, Inf) point coordinates given to nearestKSearch!"' failed.

void SHOT_descriptores(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& keypoints, pcl::PointCloud<pcl::SHOT352>::Ptr &descriptores){
	pcl::SHOTEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352> shot;
	pcl::PointCloud<pcl::Normal>::Ptr normales(new pcl::PointCloud<pcl::Normal>());

	obtenerNormales(keypoints, normales);

	shot.setRadiusSearch(0.05);
	shot.setInputCloud(keypoints);
	shot.setInputNormals(normales);
	shot.compute(*descriptores);

	cout << "Numero de descriptores SHOT: " << descriptores->size() << "\n";	
}*/

//--------------------------------------------


//void Matching(pcl::PointCloud<pcl::PFHSignature125>::Ptr &descriptores){
void Matching(pcl::PointCloud<pcl::FPFHSignature33>::Ptr &descriptores){
//void Matching(pcl::PointCloud<pcl::SHOT352>::Ptr &descriptores){
//void Matching(pcl::PointCloud<pcl::VFHSignature308>::Ptr &descriptores){
	//pcl::registration::CorrespondenceEstimation<pcl::PFHSignature125, pcl::PFHSignature125> ce;
	pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> ce;
	//pcl::registration::CorrespondenceEstimation<pcl::VFHSignature308, pcl::VFHSignature308> ce;
	//pcl::registration::CorrespondenceEstimation<pcl::SHOT352, pcl::SHOT352> ce;
	ce.setInputSource(descriptores);
	ce.setInputTarget(descriptores_anteriores);
	ce.determineCorrespondences(*correspondenciaEstimada);
	std::cout << "Numero de correspondencias encontradas: " << correspondenciaEstimada->size() << std::endl;
}

Eigen::Matrix4f RANSAC(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &keypoints, pcl::CorrespondencesPtr mejorCorrespondencia){
	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB> crsc;

    crsc.setInputSource(keypoints);
    crsc.setInputTarget(nube_key_anterior);

	crsc.setRefineModel(true);
    crsc.setInlierThreshold(0.50);
    crsc.setMaximumIterations(10000);
    crsc.setInputCorrespondences(correspondenciaEstimada);
	crsc.getCorrespondences(*mejorCorrespondencia);
	return crsc.getBestTransformation();
}

void ICP(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &nube_input){
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    pcl::PointCloud<pcl::PointXYZRGB> nube_alineada;

    icp.setInputSource(nube_input);
    icp.setInputTarget(nube_filtrada_anterior);

    icp.setMaxCorrespondenceDistance(0.05);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.align(nube_alineada);
	transformacion_actual = icp.getFinalTransformation();

	std::cout << "ICP Fitness Score: " << icp.getFitnessScore() << "\n";
}

void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr nube_capturada(new pcl::PointCloud<pcl::PointXYZRGB>(*msg));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr nube_filtrada(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr normales(new pcl::PointCloud<pcl::Normal>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZRGB>());
	//pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptores(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptores(new pcl::PointCloud<pcl::FPFHSignature33>());
	//pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptores(new pcl::PointCloud<pcl::VFHSignature308>()); //no conseguido
	//pcl::PointCloud<pcl::SHOT352>::Ptr descriptores(new pcl::PointCloud<pcl::SHOT352>()); //no conseguido

	cout << "Puntos capturados: " << nube_capturada->size() << endl;

	eliminarNaN(nube_capturada);

	cout << "Puntos filtrados: " << nube_capturada->size() << endl;

	//VoxelGrid
	pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;
	voxelGrid.setInputCloud(nube_capturada);
	voxelGrid.setLeafSize (0.01, 0.01, 0.01);
	voxelGrid.filter(*nube_filtrada);
	cout << "Puntos tras VG: " << nube_filtrada->size() << endl;

	//Normales
	resolucionNube = obtenerResolucion(nube_filtrada);
	obtenerNormales(nube_filtrada, normales);

	//Keypoints
	auto start = std::chrono::high_resolution_clock::now();
	ISS_keypoints(nube_filtrada, keypoints); //mejor
	//SIFT_keypoints(nube_filtrada, keypoints);
	//Harris_keypoints(nube_filtrada, keypoints);
	auto end = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	std::cout << "Tiempo de ejecución keypoints: " << duration.count() / 1000.0 << " segundos" << std::endl;
	//*nube_filtrada += *keypoints; //para ver keypoints de verde

	//Descriptores
	start = std::chrono::high_resolution_clock::now();
	//PFH_descriptores(nube_capturada, keypoints, descriptores);
	FPFH_descriptores(nube_capturada, keypoints, descriptores);
	//CVFH_descriptores(nube_capturada, keypoints, descriptores);
	//SHOT_descriptores(keypoints, descriptores);
	end = std::chrono::high_resolution_clock::now();
	duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	std::cout << "Tiempo de ejecución descriptores: " << duration.count() / 1000.0 << " segundos" << std::endl;

	//Matching, Ransac e ICP
	if(!nube_filtrada_anterior->empty()){ //no podemos hacer matching con solo 1 nube, necesitamos 2
		start = std::chrono::high_resolution_clock::now();
		Matching(descriptores);
		end = std::chrono::high_resolution_clock::now();
		duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
		std::cout << "Tiempo de ejecución matching: " << duration.count() / 1000.0 << " segundos" << std::endl;

		pcl::CorrespondencesPtr correspondencias(new pcl::Correspondences());

		start = std::chrono::high_resolution_clock::now();
		transformacion_actual = RANSAC(keypoints, correspondencias);
		end = std::chrono::high_resolution_clock::now();
		duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
		std::cout << "Tiempo de ejecución RANSAC: " << duration.count() / 1000.0 << " segundos" << std::endl;

		transformacion_global = transformacion_actual * transformacion_global;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr nube_con_transformacion(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::transformPointCloud(*nube_filtrada, *nube_con_transformacion, transformacion_actual);
		start = std::chrono::high_resolution_clock::now();
		ICP(nube_con_transformacion);
		end = std::chrono::high_resolution_clock::now();
		duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
		std::cout << "Tiempo de ejecución ICP: " << duration.count() / 1000.0 << " segundos" << std::endl;

		pcl::transformPointCloud(*nube_filtrada, *nube_con_transformacion, transformacion_global);
		*nube_visualizada += *nube_con_transformacion;
	}	

	*nube_filtrada_anterior  = *nube_filtrada;
	*nube_key_anterior = *keypoints;
	*descriptores_anteriores = *descriptores;
}

void simpleVis(){
  	pcl::visualization::CloudViewer viewer ("3D Mapping");
	while(!viewer.wasStopped()){
		viewer.showCloud(nube_visualizada);
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	}
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sub_pcl");
    ros::NodeHandle camera;
    ros::Subscriber sub = camera.subscribe<pcl::PointCloud<pcl::PointXYZRGB>>("/camera/depth/points", 1, callback);

    boost::thread t(simpleVis);
	transformacion_global.setIdentity();

    while(ros::ok()){
        ros::spinOnce();
    }
}