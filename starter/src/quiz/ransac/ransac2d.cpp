/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <cmath>
#include <Eigen/Dense>

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
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	while (maxIterations--) {
		std::unordered_set<int> inliers;
		while (inliers.size() < 2) 
			inliers.insert(rand() % (cloud->points.size()));
		
		float x1, y1, x2, y2;
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;

		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;

		float a, b, c;
		a = y1 - y2;
		b = x2 - x1;
		c = x1 * y2 - x2 * y1;

		for (int i = 0; i < cloud->points.size(); i++) {
			pcl::PointXYZ& p = cloud->points[i];
			float x3, y3;
			x3 = p.x; y3 = p.y;

			float d = std::fabs(a * x3 + b * y3 + c) / std::sqrt(a * a + b * b);
			if (d <= distanceTol)
				inliers.insert(i);
		}

		if (inliers.size() > inliersResult.size())
			inliersResult.swap(inliers);
	}

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	while (maxIterations--) {
		std::unordered_set<int> inliers;
		while (inliers.size() < 3) 
			inliers.insert(rand() % (cloud->points.size()));
		
		pcl::PointXYZ p1, p2, p3;
		auto itr = inliers.begin();
		p1 = cloud->points[*itr]; itr++;
		p2 = cloud->points[*itr]; itr++;
		p3 = cloud->points[*itr]; 

		Eigen::Vector3f v1 = p2.getVector3fMap() - p1.getVector3fMap();
		Eigen::Vector3f v2 = p3.getVector3fMap() - p1.getVector3fMap();
		Eigen::Vector3f v1xv2 = v1.cross(v2);

		float a, b, c, d;
		a = v1xv2[0];
		b = v1xv2[1];
		c = v1xv2[2];
		d = -(a * p1.x + b * p1.y + c * p1.z);

		for (int i = 0; i < cloud->points.size(); i++) {
			pcl::PointXYZ& p = cloud->points[i];

			float d = std::fabs(a * p.x + b * p.y + c * p.z + d) / std::sqrt(a * a + b * b + c * c + d * d);
			if (d <= distanceTol)
				inliers.insert(i);
		}

		if (inliers.size() > inliersResult.size())
			inliersResult.swap(inliers);
	}
	
	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.5);

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
