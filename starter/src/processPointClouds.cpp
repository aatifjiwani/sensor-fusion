// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr filtered_cloud (new pcl::PointCloud<PointT>);

    typename pcl::VoxelGrid<PointT> filter;
    filter.setInputCloud(cloud);
    filter.setLeafSize(filterRes, filterRes, filterRes);
    filter.filter(*filtered_cloud);

    typename pcl::PointCloud<PointT>::Ptr region_cloud (new pcl::PointCloud<PointT>);

    typename pcl::CropBox<PointT> box_filter(true);
    box_filter.setMin(minPoint);
    box_filter.setMax(maxPoint);
    box_filter.setInputCloud(filtered_cloud);
    box_filter.filter(*region_cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return region_cloud;

}

/**************************** RANSAC Planar Segmentation ************************************/


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    typename pcl::PointCloud<PointT>::Ptr road(new pcl::PointCloud<PointT>());
    for (int index : inliers->indices)
        road->points.push_back(cloud->points[index]);

    typename pcl::PointCloud<PointT>::Ptr obstacles(new pcl::PointCloud<PointT>());
    extract.setNegative(true);
    extract.filter(*obstacles);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, road);
    return segResult;
}

template<typename PointT>
std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;	

	// For max iterations 
	while (maxIterations--) {
		std::unordered_set<int> inliers;
		while (inliers.size() < 3) 
			inliers.insert(rand() % (cloud->points.size()));
		
		PointT p1, p2, p3;
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
			PointT& p = cloud->points[i];

			float distance = std::fabs(a * p.x + b * p.y + c * p.z + d) / std::sqrt(a * a + b * b + c * c);
			if (distance <= distanceTol)
				inliers.insert(i);
		}

		if (inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}
	
	return inliersResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    /** PCL Implementation */

    // pcl::SACSegmentation<PointT> seg;
    // seg.setOptimizeCoefficients(true);
    // seg.setModelType(pcl::SACMODEL_PLANE);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setMaxIterations(maxIterations);
    // seg.setDistanceThreshold(distanceThreshold);

    // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    // pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    // seg.setInputCloud(cloud);
    // seg.segment(*inliers, *coefficients);

    /** My implementation */

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    std::unordered_set<int> indices = RansacPlane<PointT>(cloud, maxIterations, distanceThreshold);
    inliers->indices.insert(inliers->indices.end(), indices.begin(), indices.end());

    if (inliers->indices.size() == 0)
        std::cerr << "No plane found" << std::endl;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

/**************************** Euclidean Clustering ************************************/
namespace {
    void proximity(int point, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& visited, KdTree* tree, float distanceTol) {
        if (!visited[point]) {
            visited[point] = true;
            cluster.push_back(point);

            std::vector<int> nearby = tree->search(points[point], distanceTol);
            for (int n_point : nearby) {
                proximity(n_point, points, cluster, visited, tree, distanceTol);
            }
        }
    }

    std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
    {
        std::vector<std::vector<int>> clusters;
        std::vector<bool> visited(points.size(), false);

        for (int point = 0; point < points.size(); point++) {
            if (!visited[point]) {
                // point has not been processed

                // create new cluster
                std::vector<int> cluster;
                proximity(point, points, cluster, visited, tree, distanceTol);
                clusters.push_back(cluster);
            }
        }
    
        return clusters;
    }
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    /** PCL Implementation */

    // typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    // tree->setInputCloud(cloud);

    // std::vector<pcl::PointIndices> cluster_indices;
    // pcl::EuclideanClusterExtraction<PointT> ec;
    // ec.setClusterTolerance(clusterTolerance);
    // ec.setMinClusterSize(minSize);
    // ec.setMaxClusterSize(maxSize);
    // ec.setSearchMethod(tree);
    // ec.setInputCloud(cloud);
    // ec.extract(cluster_indices);

    /** My implementation */
    std::vector<std::vector<float>> points;
    KdTree* tree = new KdTree();

    int index = 0;
    for (auto& pt : cloud->points) {
        points.push_back(std::vector<float>{pt.x, pt.y, pt.z});
        tree->insert(points.back(), index++);
    }

    std::vector<std::vector<int>> cluster_indices = euclideanCluster(points, tree, clusterTolerance);

    for (const auto& cluster : cluster_indices) {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (const auto& idx : cluster) //.indices)
            cloud_cluster->push_back((*cloud)[idx]);

        if ((cloud_cluster->size() < minSize) || (cloud_cluster->size() > maxSize))
            continue;

        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

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
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

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