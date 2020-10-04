//* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <string>

using namespace std;

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

// Demo PointCloud

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;

    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 

    double setGroundSlope = 0;
    string cloud_name = "my_cloud";

    Lidar* lidar = new Lidar(cars,setGroundSlope); // Create Lidar sensor
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud = lidar->scan();
    //renderRays(viewer,lidar->position,lidar_cloud);
    //renderPointCloud(viewer,lidar_cloud,cloud_name);

    // TODO:: Create point processor
    
    // ProcessPointClouds<pcl::PointXYZ> pointProcessor; // Stack (Static objects)
    ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>(); // Heap (Dynamic objects)
    
    int iterations = 100; 
    float distance_threshold = 0.2; 

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(lidar_cloud, iterations, distance_threshold); 
    //renderPointCloud(viewer,segmentCloud.first,"obstacle_cloud",Color(1,0,0)); // Obstacle cloud
    renderPointCloud(viewer,segmentCloud.second,"road_cloud",Color(0,1,0)); // Road cloud

    // Perform clustering

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first,1.0,3,30);

    int clusterId = 0;
    std::vector<Color> colours = {Color(1,0,0),Color(0,1,1),Color(0,0,1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "Cluster size: ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colours[clusterId%colours.size()]);
        
        Box box_cluster = pointProcessor->BoundingBox(cluster);
        renderBox(viewer,box_cluster,clusterId);

        clusterId++;
    }
}

// Carlos Gomez Huelamo - My funtions

// Real-world PCD analysis

void cityBlock (pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    bool downsampling_cropping = true;
    bool my_3D_RANSAC = true;
    bool my_3D_Clustering = true;
    bool filter_boxes = true;
    bool smallest_fitting_box = false;

    auto start_time = std::chrono::steady_clock::now();

    // If single PCD is analyzed:
    
    //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    //pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    // 1. Downsample the point cloud using Voxel Grid and Region-based filtering

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud;

    if (downsampling_cropping)
    {
        float voxel_size = 0.32; // m
        filtered_cloud = pointProcessorI->FilterCloud(input_cloud, voxel_size, Eigen::Vector4f (-8.0,-5.0,-2.1,1), Eigen::Vector4f (24.0,6.5,1.0,1));
    }
    else
    {
        filtered_cloud = input_cloud;  
    }
    
    // 2. Segment using RANSAC 3D 
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_inliers (new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_outliers (new pcl::PointCloud<pcl::PointXYZI>());
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud;
    std::unordered_set<int> inliers;

    if (my_3D_RANSAC)
    {
        int iterations = 60;
        float distance_tolerance = 0.3;

        inliers = pointProcessorI->RANSAC_3D(filtered_cloud, iterations, distance_tolerance);

        for(int index = 0; index < filtered_cloud->points.size(); index++)
        {
            pcl::PointXYZI point = filtered_cloud->points[index];

            if(inliers.count(index))
                cloud_inliers->points.push_back(point);
            else
                cloud_outliers->points.push_back(point);
        }

        if(inliers.size())
        {
            renderPointCloud(viewer,cloud_inliers,"inliers",Color(0,1,0)); // Road point cloud
            //renderPointCloud(viewer,cloud_outliers,"outliers",Color(1,1,0)); // Non-road point cloud
        }
        else
        {
            renderPointCloud(viewer,filtered_cloud,"filtered_cloud"); 
        }
    }
    else
    {
        int iterations = 100; 
        float distance_threshold = 0.2; 

        segmentCloud = pointProcessorI->SegmentPlane(filtered_cloud, iterations, distance_threshold);
        
        renderPointCloud(viewer,segmentCloud.first,"obstacle_cloud",Color(1,0,0)); // Non-road point cloud
        renderPointCloud(viewer,segmentCloud.second,"road_cloud",Color(0,1,0)); // Road point cloud
    }
     
    // 3. Perform clustering to identify relevant obstacles

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_clusters;

    float min_distance_clusters = 1.1;
    int min_cluster_size = 8;
    int max_cluster_size = 400;

    if (my_3D_Clustering)
    {
        cloud_clusters = pointProcessorI->Clustering_3D(cloud_outliers,min_distance_clusters,min_cluster_size,max_cluster_size);
    } 
    else
    {
        cloud_clusters = pointProcessorI->Clustering(segmentCloud.first,min_distance_clusters,min_cluster_size,max_cluster_size); // pcl built-in functions 
    }

    int cluster_id = 0;

    std::vector<Color> colours = {Color(1,0,0),Color(1,1,0),Color(0,0,1)};

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloud_clusters)
    {
        //std::cout << "Cluster size: ";
        //pointProcessorI->numPoints(cluster);

        if (smallest_fitting_box) // Smallest fitting box
        {
            BoxQ box_cluster = pointProcessorI->BoundingBoxQ(cluster);     
        }
        else // Standard bounding box
        {
            Box box_cluster = pointProcessorI->BoundingBox(cluster);

            float length = box_cluster.x_max - box_cluster.x_min;
            float width  = box_cluster.y_max - box_cluster.y_min;
            float height = box_cluster.z_max - box_cluster.z_min;

            if (filter_boxes)
            {
                //if (((length >= 1.0 && length < 5) && (width >= 0.8 && width < 2) && (height >= 0.8 && height < 2.2))  // Cars/Truck
                //||  ((length < 0.5) && (width < 0.5) && (height >= 0.5))) // Pole
                if (width > 3.0 || height < 0.5 || length/width > 5)
                {
                    continue;
                }
                else
                {
                    renderPointCloud(viewer,cluster,"Obstacle Cloud"+std::to_string(cluster_id),colours[cluster_id%colours.size()]);
                    renderBox(viewer,box_cluster,cluster_id);
                }
            }
            else
            {
                renderPointCloud(viewer,cluster,"Obstacle Cloud"+std::to_string(cluster_id),colours[cluster_id%colours.size()]);
                renderBox(viewer,box_cluster,cluster_id);
            }           
        }
        
        cluster_id++;
    }

    auto end_time = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "Downsampling-Cropping + Segmentation + Clustering took: " << elapsed_time.count() << " ms" << std::endl << std::endl;
}

// Carlos Gomez Huelamo - End of My funtions

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-17, 0, 6, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

int main (int argc, char** argv)
{
    std::cout << "Starting enviroment ..." << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    
    //simpleHighway(viewer); // Demo PointCloud

    // Stream PCD

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto stream_iterator = stream.begin(); // Iterator
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer

        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process

        inputCloudI = pointProcessorI->loadPcd((*stream_iterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        // Go ahead in the folder for a new pcd file

        stream_iterator++; // As a pointer, you have to increase the memory address

        if (stream_iterator == stream.end())
        {
            stream_iterator = stream.begin(); // Restart. So we finish the viewer if the user
            // press CTRL+C, but if we reach the last pcd file of the folder, we restart
        }

        viewer->spinOnce ();
    } 
}