// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set> // Used for RANSAC 3D function

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    auto start_time = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>());

    // Create the filtering object (Based on Voxel Grid Method)

    typename pcl::VoxelGrid<PointT> voxel_grid_object;
    voxel_grid_object.setInputCloud(cloud);
    voxel_grid_object.setLeafSize(filterRes,filterRes,filterRes); // Voxel size 
    voxel_grid_object.filter(*cloud_filtered); // Downsampled cloud

    // Use CropBox functions to create a 3D region (anything outside that region will be removed)

    typename pcl::PointCloud<PointT>::Ptr cloud_region (new pcl::PointCloud<PointT>());

    pcl::CropBox<PointT> crop_box_region(true); // If true, we can the indices of points being removed
    crop_box_region.setMin(minPoint); // Minimum corner
    crop_box_region.setMax(maxPoint); // Maximum corner
    crop_box_region.setInputCloud(cloud_filtered);
    crop_box_region.filter(*cloud_region); // Downsample and cropped cloud

    // Remove the points of the roof of the ego-vehicle

    std::vector<int> indices;

    pcl::CropBox<PointT> roof_region(true); 
    roof_region.setMin(Eigen::Vector4f (-1.5,-1.7,-1,-1));
    roof_region.setMax(Eigen::Vector4f (2.6,1.7,-0.4,1));
    roof_region.setInputCloud(cloud_region);
    roof_region.filter(indices); // Get the indices of points being removed 

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());

    // Set inliers based on extracted indices

    for (int i : indices)
    {
        inliers->indices.push_back(i);
    }

    // Extract points based on inliers

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_region);
    extract.setIndices(inliers); // Roof points
    extract.setNegative(true); 
    extract.filter(*cloud_region);

    auto end_time = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "Downsampling-Cropping: " << elapsed_time.count() << " ms " << std::endl;

    return cloud_region;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    // Here it is considered that the plane is the road

    // Create two point clouds

    typename pcl::PointCloud<PointT>::Ptr obstacle_cloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr road_cloud (new pcl::PointCloud<PointT>());

    // Looping over the inlier indices

    for (int i : inliers->indices) 
    {
        road_cloud->points.push_back(cloud->points[i]);
    }

    // Create the filtering object

    typename pcl::ExtractIndices<PointT> extract;

    // Extract the inliers

    extract.setInputCloud(cloud); // Second argument
    extract.setIndices(inliers); // First argument
    extract.setNegative(true);
    extract.filter(*obstacle_cloud); 

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle_cloud, road_cloud);
    return segResult;
}

// Carlos Gomez Huelamo - My funtions 

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RANSAC_3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    auto start_time = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult; 

	while(maxIterations--)
	{
		// We randomly pick three points in order to create our plane

		std::unordered_set<int> plane_points;

		while (plane_points.size() < 3)  // Three points
		{
			plane_points.insert(rand()%(cloud->points.size())); 
		}

		float x1,y1,z1,x2,y2,z2,x3,y3,z3; // line_points 3D coordinates

		auto itr = plane_points.begin(); 

		x1 = cloud->points[*itr].x;  // Reference point  
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++; 
		x2 = cloud->points[*itr].x;  
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++; 
		x3 = cloud->points[*itr].x;  
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		float a,b,c,d,e,f;

		a = x2-x1;
		b = y2-y1;
		c = z2-z1;
		d = x3-x1;
		e = y3-y1;
		f = z3-z1;

		// Plane coefficients

		float A,B,C,D;

		A = b*f-c*e;
		B = c*d-a*f;
		C = a*e-b*d;
		D = -(A*x1+B*y1+C*z1);

		for (int i=0; i<cloud->points.size(); i++)
		{
			if (plane_points.count(i)>0) 
			{
				continue;
			}

			PointT point = cloud->points[i]; 

			float x4 = point.x; 
			float y4 = point.y;
			float z4 = point.z;

			float d = fabs(A*x4+B*y4+C*z4+D)/sqrt(pow(A,2)+pow(B,2)+pow(C,2));

			if (d <= distanceTol)
			{
				plane_points.insert(i);
			}
		}

		if (plane_points.size() > inliersResult.size())
		{
			inliersResult = plane_points; 
		}
	}

    auto end_time = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "Segmentation: " << elapsed_time.count() << " ms" << std::endl;

	return inliersResult;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering_3D(typename pcl::PointCloud<PointT>::Ptr cloud, float cluster_tolerance, int min_size, int max_size)
{
    // Time clustering process
    auto start_time = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    // Create KdTree object, Euclidean Cluster Extraction (ECC) object and cluster indices

    KdTree* tree = new KdTree; 
    std::vector<pcl::PointIndices> cloud_indices;
    std::vector<std::vector<float>> points;

    // Fill KDTree based on cloud points

    for (int i=0; i<cloud->points.size (); i++) 
    {
        std::vector<float> point_3D = {cloud->points[i].x,cloud->points[i].y,cloud->points[i].z};
        points.push_back(point_3D);
    	tree->insert(point_3D,i);
    }

    // Get clusters indices based on KDTree, points, tolerance, minimum and maximum cluster size

    std::vector<std::vector<int>> clusters_indices = euclidean_cluster_3D(points, tree, cluster_tolerance, min_size, max_size);

    // Get individual clusters based on extracted clusters_indices

    int cnt = 0;

    for (std::vector<int> cluster_index : clusters_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for (int index : cluster_index)
        {
            cloudCluster->points.push_back(cloud->points[index]);
        }

        // Evaluate cluster size

        if ((cloudCluster->points.size() > min_size) && (cloudCluster->points.size() <= max_size))
        {
            cloudCluster->width = cloudCluster->points.size();
            cloudCluster->height = 1;
            cloudCluster->is_dense = true;

            clusters.push_back(cloudCluster);
        } 
    }

    auto end_time = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "Clustering: " << elapsed_time.count() << " ms" << std::endl;
        
    return clusters;
}

template<typename PointT>
void ProcessPointClouds<PointT>::cluster_aux_3D(int indice, const std::vector<std::vector<float>> points, std::vector<int>& cluster, std::vector<bool>& processed_points, KdTree* tree, float distanceTol)
{
    processed_points[indice] = true;
	cluster.push_back(indice);

	std::vector<int> nearest_indices = tree->search(points[indice],distanceTol); // Get indices of nearby points

	for (int id : nearest_indices)
	{
		if (!processed_points[id]) // If this point was not included, store in this cluster
		{
			cluster_aux_3D(id,points,cluster,processed_points,tree,distanceTol);
		}
	}
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclidean_cluster_3D(const std::vector<std::vector<float>> points, KdTree* tree, float distance_tolerance, int min_size, int max_size)
{
	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;

	std::vector<bool> processed_points(points.size(),false); 

	int i = 0;

	while (i < points.size())
	{
		if (processed_points[i])
		{
			i++;
			continue; 
		}

		std::vector<int> cluster; // New cluster
		cluster_aux_3D(i,points,cluster,processed_points,tree,distance_tolerance); 

		clusters.push_back(cluster);
		i++;
	}
 
	return clusters;
}

template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Computed smallest fitting box oriented with the XY plane (assuming flat plane)

    // 1. Compute principal directions

    Eigen::Vector4f pca_centroid;
    pcl::compute3DCentroid(*cluster, pca_centroid);

    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cluster, pca_centroid, covariance);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    // 2. Transform the original cloud to the origin where the principal components correspond
    //    to the axes

    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pca_centroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);

    // 3. Get the minimum and maximum points of the transformed cloud

    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // 4. Final transform

    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); 
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pca_centroid.head<3>();

    // 5. Set rotation around x and y equal to 0

    std::cout << "Quaternion x: " << bboxQuaternion.x() << std::endl;
    std::cout << "Quaternion y: " << bboxQuaternion.y() << std::endl;
    std::cout << "Quaternion z: " << bboxQuaternion.z() << std::endl;
    std::cout << "Quaternion w: " << bboxQuaternion.w() << std::endl;

    // Fill boxQ structure 

    BoxQ boxQ;

    boxQ.bboxTransform  = bboxTransform;
    boxQ.bboxQuaternion = bboxQuaternion;
    boxQ.cube_length    = maxPoint.x - minPoint.x;
    boxQ.cube_width     = maxPoint.y - minPoint.y;
    boxQ.cube_height    = maxPoint.z - minPoint.z;

    return boxQ;
}

// Carlos Gomez Huelamo - End of My funtions

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud, in order to segment cloud
    // in two parts, the driveable plane and obstacles

    pcl::SACSegmentation<PointT> seg; // Segmentation object
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices}; 
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

    // Setting segmentation object parameters

    seg.setOptimizeCoefficients(true); // Optional
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planer component from the input cloud

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        std::cout<<"Could not estimate a planar model for the given dataset."<<std::endl;
    }
    
    // std::cout<<"Number of inliers: "<<inliers->indices.size()<<std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Segmentation took: " << elapsedTime.count() << " ms" << std::endl;

    return segResult;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    // Create KdTree object, Euclidean Cluster Extraction (ECC) object and cluster indices

    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>()); 
    std::vector<pcl::PointIndices> cloud_indices;
    
    //pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    typename pcl::EuclideanClusterExtraction<PointT> ec; // Set generic PointT here?

    // Set input cloud and ECC parameters parameters

    tree->setInputCloud(cloud); // In this case obstacle_cloud
    ec.setClusterTolerance(clusterTolerance);  // in m (so 0.02 -> 2 cm)
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cloud_indices);

    // Get individual clusters based on extracted cloud_indices

    int cnt = 0;
    for (pcl::PointIndices getIndices : cloud_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for (int index : getIndices.indices)
        {
            cloudCluster->points.push_back(cloud->points[index]);
        }

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Clustering took: " << elapsedTime.count() << " ms ";// << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    
    //std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}