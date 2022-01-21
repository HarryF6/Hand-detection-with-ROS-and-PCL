#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <pcl/io/pcd_io.h>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <visualization_msgs/Marker.h>


//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/console/time.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <boost/algorithm/string.hpp>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/correspondence.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/features/board.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
//unuiform sampling

#include <math.h>
//liberia harris3d
#include <pcl/keypoints/harris_3d.h>
//libreria sift
#include <pcl/keypoints/sift_keypoint.h>
//para el calculo de pitch,raw,roll.
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
//unuiform sampling

#include <pcl/features/fpfh.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <tf/LinearMath/Transform.h>

//ros
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
//#include <tf2/LinearMath/Transform.h>


using namespace sensor_msgs;
using namespace std;
using namespace message_filters;
using namespace cv;
using namespace image_transport;
using namespace pcl;
using namespace geometry_msgs;



/*Funciones auxiliares para el muestreo de dos nubes de puntos*/
void spawnViewer_color(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
{

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));

	//addCoordinateSystem añade un sistema de referencia en la posicion 0,0,0
	//el parametro 1.0 indica la escala, es decir, como estan divididos los ejes.
	viewer->addCoordinateSystem (1.0);
	//rgb indica que la nube de puntos tiene que ser rgb
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointCloud);
	//con addPointCloud lo que hacemos es mostrar los puntos de la nube
	viewer->addPointCloud<pcl::PointXYZRGB> (pointCloud, rgb, "id0");
		//se queda ejecutando mientras nosotros podemos pulsar teclas y tal
		//y las procesa
		while(!viewer->wasStopped ()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	viewer->close();
}

void spawnViewer_juntar2nubes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr& pointCloud2)
{

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));

	//addCoordinateSystem añade un sistema de referencia en la posicion 0,0,0
	//el parametro 1.0 indica la escala, es decir, como estan divididos los ejes.
	viewer->addCoordinateSystem (1.0);
	//rgb indica que la nube de puntos tiene que ser rgb
	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(pointCloud);
	//con addPointCloud lo que hacemos es mostrar los puntos de la nube
	viewer->addPointCloud<pcl::PointXYZRGB> (pointCloud, "id0");
	viewer->addPointCloud<pcl::PointXYZ> (pointCloud2, "id1");

		//se queda ejecutando mientras nosotros podemos pulsar teclas y tal
		//y las procesa
		while(!viewer->wasStopped ()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	viewer->close();
}
//Mostar nube
void showCloud_color(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
	// Abrir un visualizador que muestre la nube filtrada
	//crea un hilo que va a ejecutar la funcion spawnViewer para la nube cloud.
	boost::thread visualizer = boost::thread(spawnViewer_color, cloud);
	cout << "Pulsa para continuar" << endl;
	cin.get();
	visualizer.interrupt();
	visualizer.join();
}

void mostrar_2nubes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2)
{
	// Abrir un visualizador que muestre la nube filtrada
	// crea un hilo que va a ejecutar la funcion spawnViewer para la nube cloud.
	boost::thread visualizer = boost::thread(spawnViewer_juntar2nubes, cloud,cloud2);
	cout << "Pulsa para continuar" << endl;
	cin.get();
	visualizer.interrupt();
	visualizer.join();
}

void showCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
	// Abrir un visualizador que muestre la nube filtrada
	// crea un hilo que va a ejecutar la funcion spawnViewer para la nube cloud.
	boost::thread visualizer = boost::thread(spawnViewer_color, cloud);
	cout << "Pulsa para continuar" << endl;
	cin.get();
	visualizer.interrupt();
	visualizer.join();
}

void spawnViewer_color_juntar2nubes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud2)
{

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));

	//addCoordinateSystem añade un sistema de referencia en la posicion 0,0,0
	//el parametro 1.0 indica la escala, es decir, como estan divididos los ejes.
	viewer->addCoordinateSystem (1.0);
	//rgb indica que la nube de puntos tiene que ser rgb
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointCloud);
	//con addPointCloud lo que hacemos es mostrar los puntos de la nube
	viewer->addPointCloud<pcl::PointXYZRGB> (pointCloud, rgb, "nube_escena");
	viewer->addPointCloud<pcl::PointXYZRGB> (pointCloud2, rgb, "alineado");

		//se queda ejecutando mientras nosotros podemos pulsar teclas y tal
		//y las procesa
		while(!viewer->wasStopped ()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	viewer->close();
}

void showCloud_color_2nubes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud2)
{
	// Abrir un visualizador que muestre la nube filtrada
	//crea un hilo que va a ejecutar la funcion spawnViewer para la nube cloud.
	boost::thread visualizer = boost::thread(spawnViewer_color_juntar2nubes, cloud,cloud2);
	cout << "Pulsa para continuar" << endl;
	cin.get();
	visualizer.interrupt();
	visualizer.join();
}

void spawnViewer_emparejamientos(pcl::PointCloud<pcl::PointXYZRGB>::Ptr nubeKey_escena,pcl::PointCloud<pcl::PointXYZRGB>::Ptr nubeKey_obj ,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_escena,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_objeto,pcl::Correspondences corr){

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_escena);
    viewer->addPointCloud (cloud_escena,rgb, "nube_esc_cloud");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr off_nube_esc_nube_obj (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr off_nube_esc_nube_obj_keypoints (new pcl::PointCloud<pcl::PointXYZRGB> ());

    //  We are translating the nube_obj so that it doesn't end in the middle of the nube_esc representation
    pcl::transformPointCloud (*cloud_objeto, *off_nube_esc_nube_obj, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
    pcl::transformPointCloud (*nubeKey_obj, *off_nube_esc_nube_obj_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> off_nube_esc_nube_obj_color_handler (off_nube_esc_nube_obj, 255, 255, 128);
    viewer->addPointCloud (off_nube_esc_nube_obj, off_nube_esc_nube_obj_color_handler, "off_nube_esc_nube_obj");

    for (size_t j = 0; j < corr.size (); ++j)
    {
        std::stringstream ss_line;
        ss_line << "correspondence_line"  << j << endl;
        pcl::PointXYZRGB& nube_obj_point = off_nube_esc_nube_obj_keypoints->at (corr.at(j).index_query);
        pcl::PointXYZRGB& nube_esc_point = nubeKey_escena->at (corr.at(j).index_match);//query

        //  We are drawing a line for each pair of clustered correspondences found between the nube_obj and the nube_esc
        viewer->addLine<pcl::PointXYZRGB, pcl::PointXYZRGB	> (nube_obj_point, nube_esc_point, 0, 255, 0, ss_line.str ());
    }

    while(!viewer->wasStopped ()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    viewer->close();



    return;
}

 
void visualizar_emparejamientos(pcl::PointCloud<pcl::PointXYZRGB>::Ptr nubeKey_escena,pcl::PointCloud<pcl::PointXYZRGB>::Ptr nubeKey_obj ,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_escena,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_objeto,pcl::Correspondences corr)
{
	// Abrir un visualizador que muestre la nube filtrada
	//crea un hilo que va a ejecutar la funcion spawnViewer para la nube cloud.
	boost::thread visualizer = boost::thread(spawnViewer_emparejamientos, nubeKey_escena,nubeKey_obj,cloud_escena,cloud_objeto,corr);
	cout << "Pulsa para continuar" << endl;
	cin.get();
	visualizer.interrupt();
	visualizer.join();
}

/***************************ISS + SHOT********************/
double computeCloudResolution (const pcl::PointCloud<PointXYZRGB>::ConstPtr &cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<PointXYZRGB> tree;
  tree.setInputCloud (cloud);

  for (std::size_t i = 0; i < cloud->size (); ++i)
  {
    if (! std::isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}

void EliminarPlanosDominantes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud ){

    float umbral;
    for(int i = 0; i <4; i++){
        // se repite tantos planos queramos eliminar
        switch(i){
            case 0:
                umbral = 0.1;
                cout<<"Eliminamos plano con un umbral de "<< umbral<<endl;
                
                break;
            case 1:
                umbral = 0.065;
                cout<<"Eliminamos plano con un umbral de "<< umbral<<endl;
                
                break;
            case 2:
                umbral = 0.01;
                cout<<"Eliminamos plano con un umbral de "<< umbral<<endl;
                
                break;
            case 3:
                umbral = 0.008;
                cout<<"Eliminamos plano con un umbral de "<< umbral<<endl;
               
                break;
        }
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (umbral);

        seg.setInputCloud (pointCloud);
        seg.segment (*inliers, *coefficients);

        pcl::ExtractIndices<pcl::PointXYZRGB> eifilter (true); // Initializing with true will allow us to extract the removed indices
        eifilter.setInputCloud (pointCloud);
        eifilter.setIndices (inliers);
        // The resulting cloud_out contains all points of cloud_in that are indexed by indices_in
        // indices_rem = eifilter.getRemovedIndices ();
        // The indices_rem array indexes all points of cloud_in that are not indexed by indices_in

        eifilter.setNegative (true);
        eifilter.filter (*pointCloud);
        //showCloud_color(pointCloud);
    }


}




pcl::PointCloud<PointXYZRGB>::Ptr nube_mano (new pcl::PointCloud<PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr objeto_keypoints (new pcl::PointCloud<pcl::PointXYZRGB> ());
pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss_objeto;
pcl::PointCloud<pcl::Normal>::Ptr ObjetoN(new pcl::PointCloud<pcl::Normal>());
pcl::NormalEstimation<pcl::PointXYZRGB,pcl::Normal> NormalesO;
pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
pcl::PointCloud<pcl::SHOT352>::Ptr objeto_descriptors (new pcl::PointCloud<pcl::SHOT352> ());

class RegistroNubes{
    private:
      // rosrun tf static_transform_publisher 0 0 0 0 0 0  map camera_depth_optical_frame 50
        // Variables para utilizar en las funciones de la clase
        ros::NodeHandle n;
        ros::Publisher escena_pub;
        ros::Publisher mano_pub;
        ros::Publisher escena_rec_pub;
        ros::Publisher obj_rec_pub;
        ros::Publisher vis_pub;

    public:
      void callback_nubes(const sensor_msgs::PointCloud2ConstPtr& point_cloud)
    {
      static tf::TransformBroadcaster br;
      //guardamos escena
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr escena  (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr escena_copy  (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr escena1  (new pcl::PointCloud<pcl::PointXYZRGB>);
      
      //convertimos la nube de puntos a variable de la pcl
      pcl::PCLPointCloud2 pcl_pc;
      pcl_conversions::toPCL(*point_cloud,pcl_pc);
      pcl::fromPCLPointCloud2(pcl_pc,*escena);


      string etiquetaE = "/home/gfa31/Desktop/practica2/ManoData/escena.pcd";
      pcl::io::savePCDFile (etiquetaE, *escena);

      copyPointCloud(*escena, *escena_copy);
      copyPointCloud(*nube_mano, *escena1);


     	float minX = -0.5, minY = -0.7, minZ = -1.5; 
      float maxX = +0.5, maxY = +0.7, maxZ = +1.5;
      pcl::CropBox<pcl::PointXYZRGB> boxFilter;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr nube_cropBox (new pcl::PointCloud<pcl::PointXYZRGB>);
      boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
      boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
      boxFilter.setInputCloud(escena_copy);
      boxFilter.filter(*escena_copy);




      /*********************************************************/
      cout<<"Metodo ISS + SHOT ..."<<endl;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr escena_keypoints (new pcl::PointCloud<pcl::PointXYZRGB> ());
      pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss_escena;


      // Fill in the model cloud

      double model_resolutionE = computeCloudResolution (escena);
      //double model_resolutionO = computeCloudResolution (escena1);
      double iss_salient_radius_;
      double iss_non_max_radius_;
      double iss_gamma_21_ =  0.975;
      double iss_gamma_32_ = 0.975;
      double iss_min_neighbors_ = 25;
      int iss_threads_ = 4;

      // Compute model_resolution
      iss_salient_radius_ = 6 * model_resolutionE;
      iss_non_max_radius_ = 4 * model_resolutionE;

      iss_escena.setSearchMethod (tree);
      iss_escena.setSalientRadius (iss_salient_radius_);
      iss_escena.setNonMaxRadius (iss_non_max_radius_);
      iss_escena.setThreshold21 (iss_gamma_21_);
      iss_escena.setThreshold32 (iss_gamma_32_);
      iss_escena.setMinNeighbors (iss_min_neighbors_);
      iss_escena.setNumberOfThreads (iss_threads_);
      iss_escena.setInputCloud (escena_copy);
      iss_escena.compute (*escena_keypoints);

  


      std::cout << "Scene total points: " << escena_copy->size () << "; Selected Keypoints: " << escena_keypoints->size () << std::endl;


      pcl::PointCloud<pcl::Normal>::Ptr ObjetoN(new pcl::PointCloud<pcl::Normal>());
      pcl::PointCloud<pcl::Normal>::Ptr EscenaN(new pcl::PointCloud<pcl::Normal>());
      //estimamos las normales de las nubes para pasarselas a pfh.
      pcl::NormalEstimation<pcl::PointXYZRGB,pcl::Normal> NormalesO;
      pcl::NormalEstimation<pcl::PointXYZRGB,pcl::Normal> NormalesE;

      //metemos las nubes de los keypoints.

      //NormalesO.setInputCloud(escena1);
      NormalesE.setInputCloud(escena_copy);


      NormalesE.setSearchMethod(tree);
      //establecemos el radio de busqueda.

      NormalesE.setRadiusSearch(0.05); // 0.07 okay

      //calculamos las normales

      NormalesE.compute(*EscenaN);


      pcl::PointCloud<pcl::SHOT352>::Ptr escena_descriptors (new pcl::PointCloud<pcl::SHOT352> ());


      pcl::SHOTEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352> descr_est;
      descr_est.setSearchMethod(tree);
      descr_est.setRadiusSearch (0.1);
      descr_est.setKSearch(0);


      descr_est.setInputCloud (escena_keypoints);
      descr_est.setInputNormals (EscenaN);
      descr_est.setSearchSurface (escena_copy);
      descr_est.compute (*escena_descriptors);
      std::cout << "SHOT size(escena): " << escena_descriptors->size () << std::endl;
      std::cout << "SHOT size(objeto): " << objeto_descriptors->size () << std::endl;

      // rechazamos los emparejamientos malos
      // buscamos todas las correspondencias y las buenas
      pcl::registration::CorrespondenceEstimation<pcl::SHOT352,pcl::SHOT352> corresp;

      corresp.setInputSource(objeto_descriptors);
      corresp.setInputTarget(escena_descriptors);

      pcl::Correspondences emparejamientos;
      pcl::Correspondences emparejamientos_buenos;
      // Computar los emparejamientos entre los descriptores de la escena y del
      // objeto
      // determinamos todas las correspondencias calculadas
      corresp.determineCorrespondences(emparejamientos);

      pcl::registration::CorrespondenceRejectorSampleConsensus< pcl::PointXYZRGB > correspBuenas;

      correspBuenas.setInputSource(objeto_keypoints); // fuente
      correspBuenas.setInputTarget(escena_keypoints); // objetivo

      // Rechazar los emparejamientos incorrectos
      correspBuenas.getRemainingCorrespondences(emparejamientos,emparejamientos_buenos);
      std::cout << "Object total points: " << escena1->size () << "; Selected Keypoints: " << objeto_keypoints->size () << std::endl;
      std::cout << "Scene total points: " << escena_copy->size () << "; Selected Keypoints: " << escena_keypoints->size () << std::endl;

      //visualizar_emparejamientos(escena_keypoints,objeto_keypoints,escena_copy,escena1,emparejamientos_buenos);

      // Calcular la transformación que hay entre la nube de puntos del objeto y
      // del objeto en la escena
      Eigen::Matrix4f matrix; // matriz 4x4
      // obtenemos la matriz de transformacion
      matrix = correspBuenas.getBestTransformation();
      // la aplicamos
      //pcl::transformPointCloud (*objeto, *objeto, matrix);
      cout<<endl<<endl<<"Matriz de transformación RANSAC:"<<endl<<matrix<<endl;
      // Refinar el resultado con ICP
      pcl::IterativeClosestPoint<pcl::PointXYZRGB,pcl::PointXYZRGB> icp;
      //metemos la nube inicial
      icp.setInputSource(escena1); //fuente
      //metemos la nube destino
      icp.setInputTarget(escena_copy); // objetivo
      icp.setMaximumIterations (25);
      //creamos una nube que va a ser la nube resultante, que va a quedar en la posicion de la nube destino.
      //pcl::PointCloud<pcl::PointXYZRGB>::Ptr resultante (new pcl::PointCloud<pcl::PointXYZRGB>);
      //con align se llama al algoritmo que nos va a permitir alinear ambas nubes y nos devuelve en Final la nube resultante de la alineacion.
      icp.align(*escena1, matrix);
      Eigen::Matrix4f final_matrix;

      //hasConverged devuelve true si las nubes de puntos estan alineadas.
      if(icp.hasConverged() == true){

        cout<<"Tanto la escena como el objeto han congeniado."<<endl;
        cout<<"Fitness score: " <<icp.getFitnessScore() << std::endl;

                final_matrix =icp.getFinalTransformation();				
                // aplicamos la transformación 
                //pcl::transformPointCloud (*objeto, *objeto, final_matrix);
                
                //transf  = final_matrix;
                cout<<"Matriz de transformación de RANSACxICP:"<<endl<<final_matrix<<endl;

          
      }
        
      else{
        cout<<"No han congeniado la escena y el objeto."<<endl;
        // no se aplica la matriz de ICP
      }
      // mostramos ambas nubes 
      //showCloud_color_2nubes(escena_copy,escena1);
      cout << "Finalizado ISS + SHOT" << endl;

      //publicar nubes
      sensor_msgs::PointCloud2 escena_ros;
      pcl::toROSMsg(*escena_copy, escena_ros);
      sensor_msgs::PointCloud2 mano_ros;
      pcl::toROSMsg(*escena1, mano_ros);    
      ros::NodeHandle n;
      ros::Publisher escena_rec_pub = n.advertise<sensor_msgs::PointCloud2>("/escena_recortada", 1);
      ros::Publisher obj_rec_pub = n.advertise<sensor_msgs::PointCloud2>("/mano_tracked", 1);
      // escena recortada
      escena_rec_pub.publish(escena_ros);
      // mano posicionada
      obj_rec_pub.publish(mano_ros);


      /*tf::Vector3 origin;
      origin.setValue((final_matrix(0,3)),(final_matrix(1,3)),(final_matrix(2,3)));
      //static_cast<double>
      tf::Matrix3x3 tf3d;
      tf3d.setValue((final_matrix(0,0)), (final_matrix(0,1)), (final_matrix(0,2)), (final_matrix(1,0)), (final_matrix(1,1)), (final_matrix(1,2)), (final_matrix(2,0)), (final_matrix(2,1)), (final_matrix(2,2)));
      //cout<<origin<<endl;
      tf::Quaternion tfqt;
      tf3d.getRotation(tfqt);

      tf::Transform transform;
      transform.setOrigin(origin);
      transform.setRotation(tfqt);

      tf::StampedTransform msgs_tf;
      msgs_tf.setData(transform);
      msgs_tf.child_frame_id_ = "hand_frame";
      msgs_tf.stamp_ = ros::Time::now();
      msgs_tf.frame_id_ = "map";*/



      //tf del registro de pares de nubes
      //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame","hand_frame"));
      //br.sendTransform(msgs_tf);
      
      /*********************************************************/
      //escena real
      ros::Publisher escena_pub = n.advertise<sensor_msgs::PointCloud2>("/escena", 10);
      escena_pub.publish(point_cloud);
      //mano real
      ros::Publisher mano_pub = n.advertise<sensor_msgs::PointCloud2>("/mano", 10);
      sensor_msgs::PointCloud2 mano_real_ros;
      pcl::toROSMsg(*nube_mano, mano_real_ros);  
      mano_pub.publish(mano_real_ros);

      Eigen::Vector4f centroid2; 
      pcl::compute3DCentroid (*escena1, centroid2); 
 

      tf::Vector3 origin3;
      origin3.setValue(centroid2[0],centroid2[1],centroid2[2]);
      //static_cast<double>
      tf::Quaternion tfqt3;
      tfqt3.setRPY(0,-3.15,0);

      tf::Transform transform3;
      transform3.setOrigin(origin3);
      transform3.setRotation(tfqt3);

      //while (true) br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "lifting_eye"));
      //tf del registro de pares de nubes
      br.sendTransform(tf::StampedTransform(transform3, ros::Time::now(), "base_link","hand_frame"));
      //br.sendTransform(msgs_tf);

      ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "mano_marker", 10 );
      visualization_msgs::Marker marker;
      marker.header.frame_id = "base_link";
      marker.header.stamp = ros::Time();
      marker.ns = "my_namespace";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = centroid2[0];
      marker.pose.position.y = centroid2[1];
      marker.pose.position.z = centroid2[2];
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.color.a = 1.0; // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      //only if using a MESH_RESOURCE marker type:
      //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
      vis_pub.publish( marker );
      
    }


      
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "RegisterPointCloud");
  ros::NodeHandle nh;

  RegistroNubes nubes_objeto;

  //realizar pasos de la mano, extracción de caracteristicas, calculo de normales y descriptores
  //cargar nube de puntos de la mano
  std::string string_escena1 = "/home/gfa31/ws_percepcion/src/paquete_pcl/src/mano.pcd";
  pcl::PointCloud<PointXYZRGB>::Ptr escena1 (new pcl::PointCloud<PointXYZRGB>);
  // cargando la nube a color
  if(io::loadPCDFile<PointXYZRGB> (string_escena1, *escena1) == -1)
  {
    std::cerr<<"Error al cargar la nube de puntos "+string_escena1+"\n"<<std::endl;
    //return -1;
  }
  copyPointCloud(*escena1, *nube_mano); 

  //caracteristicas
  double model_resolutionO = computeCloudResolution (escena1);
  double iss_salient_radius_;
  double iss_non_max_radius_;
  double iss_gamma_21_ =  0.975;
  double iss_gamma_32_ = 0.975;
  double iss_min_neighbors_ = 15;
  int iss_threads_ = 4;
  iss_salient_radius_ = 6 * model_resolutionO;
  iss_non_max_radius_ = 4 * model_resolutionO;

  iss_objeto.setSearchMethod (tree);
  iss_objeto.setSalientRadius (iss_salient_radius_);
  iss_objeto.setNonMaxRadius (iss_non_max_radius_);
  iss_objeto.setThreshold21 (iss_gamma_21_);
  iss_objeto.setThreshold32 (iss_gamma_32_);
  iss_objeto.setMinNeighbors (iss_min_neighbors_);
  iss_objeto.setNumberOfThreads (iss_threads_);
  iss_objeto.setInputCloud (escena1);
  iss_objeto.compute (*objeto_keypoints);

  //normales
  NormalesO.setInputCloud(escena1);
  NormalesO.setSearchMethod(tree);
  //establecemos el radio de busqueda.
  NormalesO.setRadiusSearch(0.03);
  //calculamos las normales
  NormalesO.compute(*ObjetoN);

  //descriptores
  pcl::SHOTEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352> descr_est;
  descr_est.setSearchMethod(tree);
  descr_est.setRadiusSearch (0.1);
  descr_est.setKSearch(0);

  descr_est.setInputCloud (objeto_keypoints);
  descr_est.setInputNormals (ObjetoN);
  descr_est.setSearchSurface (escena1);
  descr_est.compute (*objeto_descriptors);
  std::cout << "Object total points: " << escena1->size () <<endl;
  cout << "Selected Keypoints: " << objeto_keypoints->size () << std::endl;
  std::cout << "SHOT size(objeto): " << objeto_descriptors->size () << std::endl;


  ros::Publisher escena_pub = nh.advertise<sensor_msgs::PointCloud2>("/escena", 10);
  ros::Publisher mano_pub = nh.advertise<sensor_msgs::PointCloud2>("/mano", 10);
  ros::Publisher escena_rec_pub = nh.advertise<sensor_msgs::PointCloud2>("/escena_recortada", 1);
  ros::Publisher obj_rec_pub = nh.advertise<sensor_msgs::PointCloud2>("/mano_tracked", 1);
  ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "/mano_marker", 10 );

  ros::Subscriber sub1 = nh.subscribe("/camera/depth/color/points",100,&RegistroNubes::callback_nubes, &nubes_objeto);

  ros::spin();

  return 0;

}

