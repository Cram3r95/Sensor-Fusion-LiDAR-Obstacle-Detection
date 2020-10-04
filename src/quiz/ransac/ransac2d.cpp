/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}
 

pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0 );
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult; // Hold our best inliers (store here the inliers that
	// are related to the line with the highest number of inliers), so, the best model (line in this
	// case, since we are in 2D)
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

	// maxIterations has a certain value (e.g. 50), so we are going to iterate until this value is
	// greater than 0. If maxIterations is worth 1, then we enter in while loop, since first it is
	// evaluated, and then decreased (--). However, in the next loop it will be worth 0, so we do not
	// enter 

	while(maxIterations--)
	{
		// We randomly pick two points in order to create our line

		std::unordered_set<int> line_points;

		while (line_points.size() < 2) 
		{
			line_points.insert(rand()%(cloud->points.size())); 
		}

		float x1,y1,x2,y2; // line_points 2D coordinates

		auto itr = line_points.begin(); // iterator (pointer)  that points to the content
		// of the unordered set. Note that a set only contains unique values. The iterator
		// will point to the beginning of the memory addresses occupied by the line_inliers
		// set. Then, the content of that memory address is a certain index of the point
		// cloud

		// Note that using auto automatically adapts the type itr to a iterator that points
		// to a set object

		x1 = cloud->points[*itr].x; // Dereference the iterator in order to get that content 
		y1 = cloud->points[*itr].y;

		itr++; // Since itr is actually a pointer, so a memory address, if we increment it by
		// one we will get access to the next element of the pointed set in this case

		x2 = cloud->points[*itr].x; // Dereference the iterator in order to get that content 
		y2 = cloud->points[*itr].y;

		// According to the line equation

		float a = (y1-y2); // ?
		float b = (x2-x1); // ?
		float c = (x1*y2-x2*y1);

		for (int i=0; i<cloud->points.size(); i++)
		{
			if (line_points.count(i)>0) // If this index has already been considered, either
			// the first two points or included, continue, otherwise, next index is evaluated
			// .count(index) method returns 0 if that number is included in our set of unique
			// numbers
			{
				continue;
			}

			// Evaluate a third point, to be considered as a inlier (its normal distance
			// to our line is lower or equal than our distance threshold)

			pcl::PointXYZ point = cloud->points[i];

			float x3 = point.x; // Note that we must tackle these numbers as floats
			float y3 = point.y;
			// Note that we do not calculate z since this is a 2D RANSAC example

			// fabs() -> Returns absolute value

			// Line equation = a*x+b*y+c=0. x and y can be whatever, but if a point {x,y} holds
			// on that line, the result of that equation will be equal to 0

			float d = fabs(a*x3+b*y3+c)/sqrt(pow(a,2)+pow(b,2));

			if (d <= distanceTol)
			{
				line_points.insert(i); // We insert another element to our set of integers (set
				// of point cloud indeces)
			}
		}

		// Finally, if the size of our line_inliers is greater than zero, that means we found
		// at least a line with an associated certain number of inliers

		if (line_points.size() > inliersResult.size())
		{
			inliersResult = line_points; // Note that originally the size inliersResult was 0,
			// but after the first loop this size might be 67. If the new set of inliers (including)
			// the first two points has a size greater than 67, this variable (inliersResult) will
			// be updated
		}
	}

	return inliersResult;
}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
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

			pcl::PointXYZ point = cloud->points[i];

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

	return inliersResult;
}

int main ()
{
	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 1, 1.0); // Ten iterations and a distance tolerance of 1 meter
	std::unordered_set<int> inliers = Ransac3D(cloud, 100, 0.3);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
