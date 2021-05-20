// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>



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
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT> ());

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // Create the filtering object
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);

     // create a cloud region
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT> ());

    // create ROI for pcl to focus - create a crop box - true = extract removed indices (points inside crop box)
    pcl::CropBox<PointT> regionOfInterest(true);
    // set the max and min points to define ROI
    regionOfInterest.setMax(maxPoint);
    regionOfInterest.setMin(minPoint);
    regionOfInterest.setInputCloud(cloud_filtered);
    // return the remainging ROI into the pcl - save into cloudRegion
    regionOfInterest.filter(*cloudRegion);
    
    std::vector<int> indices;

    float roofMinx = -1.8;  
    float roofMiny = -1.5; 
    float roofMinz = -1.0; 
    float roofMaxx = 3.0; 
    float roofMaxy = 1.5; 
    float roofMaxz = 0.3;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (roofMinx, roofMiny, roofMinz, 1));
    roof.setMax(Eigen::Vector4f (roofMaxx, roofMaxy, roofMaxz, 1));
    roof.setInputCloud(cloudRegion);
    // return the remaining ROI indices inside the roof boox area  into the vector
    roof.filter(indices);


    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int point : indices) {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true); 
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    for(int index : inliers->indices) {
        planeCloud->points.push_back(cloud->points[index]);
    }
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*obstCloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    std::unordered_set<int> inliersResult;
	srand(time(NULL));

	while(maxIterations--)
	{

		//Randomly pick two points 
		std::unordered_set<int> inliers;
		while (inliers.size() < 3)
			inliers.insert(rand()%(cloud->points.size()));
		float x1 , y1 , z1 , x2 , y2 , z2 , x3 , y3 , z3;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
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

		float a , b , c , d;

		a = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		b = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		c = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
		d = (-1)*(a*x1+b*y1+c*z1);

		for (int index = 0; index < cloud->points.size(); index ++)
		{
			if(inliers.count(index) > 0)
				continue;
			pcl::PointXYZI point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;

			float distance = fabs(a*x4+b*y4+c*z4+d)/sqrt(a*a+b*b+c*c);
			
			if (distance <= distanceThreshold)
				inliers.insert(index);

		}

		if(inliers.size()>inliersResult.size())
		{
			inliersResult = inliers;
		}
	}

    typename pcl::PointCloud<PointT>::Ptr cloudInliers (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers (new pcl::PointCloud<PointT>());

    for (int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if(inliersResult.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> partedClouds(cloudOutliers, cloudInliers);
    return partedClouds;
}


    /* Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl:: SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers,*coefficients);
    if(inliers->indices.size() == 0){
        std::cout << "coudlnt find planar model \n";
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
    */


/*template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indice, const std::vector<std::vector<float>> &points, std::vector<int>& cluster, std::vector<bool> &processed, KdTree* tree, float distanceTol)
{


  if(processed[indice])
    return;
  processed[indice] = true;
  cluster.push_back(indice);

  std::vector<int> nearest = tree->search(points[indice], distanceTol);

  for (int id : nearest)
  {
    if(!processed[id])
      clusterHelper(id, points, cluster, processed, tree, distanceTol);
  }
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize)
{
    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(points.size(), false);

    int i = 0;
    while(i < points.size())
    {
        if(processed[i])
        {
          i++;
          continue;
        }
        std::vector<int> cluster;
        // pass a point id, the points, cluster as reference, processed status, tree, tolerance
        clusterHelper(i, points, cluster, processed, tree, distanceTol);

        clusters.push_back(cluster);
        i++;
    }
    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(clusterTolerance); 
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for (pcl::PointIndices getIndices : clusterIndices) 
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
        for (int index : getIndices.indices)
        cloudCluster->points.push_back(cloud->points[index]);
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}
*/

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