#include "pcd_write.hpp"

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h> 
#include <pcl/common/io.h>
#include <cmath>     
#include <limits>

// Define a shortcut for the type to avoid typing this every time
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

/**
 * Converts an OpenCV 3D Point Cloud Mat and an optional color Mat to a PCL PointCloud.
 * @param pointCloud_CV Input OpenCV Mat (CV_32FC3) from reprojectImageTo3D
 * @param colorImage_CV Input OpenCV Mat (CV_8UC3) for color, must be same size as pointCloud_CV. If empty, no color is added.
 * @return A shared pointer to the created PCL PointCloud.
 */
PointCloudT::Ptr convertCVMatToPCL(const cv::Mat& pointCloud_CV, const cv::Mat& colorImage_CV) {
    // Create the PCL point cloud object
    PointCloudT::Ptr cloud(new PointCloudT);

    // Check if the input point cloud is valid and organized
    if (pointCloud_CV.empty() || pointCloud_CV.type() != CV_32FC3) {
        std::cerr << "ERROR: Input point cloud is empty or not of type CV_32FC3." << std::endl;
        return cloud;
    }

    bool hasColor = !colorImage_CV.empty() && colorImage_CV.size() == pointCloud_CV.size();

    // Set the PCL cloud to be organized (same width and height as the image)
    cloud->width = pointCloud_CV.cols;
    cloud->height = pointCloud_CV.rows;
    cloud->is_dense = false; // Because it will have NaN/inf points we need to filter
    cloud->points.resize(cloud->width * cloud->height);

    // Iterate through every pixel in the point cloud
    for (int v = 0; v < pointCloud_CV.rows; ++v) {       // y coordinate
        for (int u = 0; u < pointCloud_CV.cols; ++u) {   // x coordinate

            // Get the 3D point from the OpenCV Mat
            cv::Vec3f point = pointCloud_CV.at<cv::Vec3f>(v, u);
            // Calculate the index for the PCL point cloud
            size_t index = v * pointCloud_CV.cols + u;

            // Check if the point is valid (finite)
            if (std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2])) {
                // Set the coordinates
                cloud->points[index].x = point[0];
                cloud->points[index].y = point[1];
                cloud->points[index].z = point[2];

                // Set the color if available
                if (hasColor) {
                    cv::Vec3b bgr = colorImage_CV.at<cv::Vec3b>(v, u);
                    // PCL uses RGB stored in a uint32_t
                    cloud->points[index].r = bgr[2]; // R from OpenCV's BGR
                    cloud->points[index].g = bgr[1]; // G
                    cloud->points[index].b = bgr[0]; // B
                }
            } else {
                // Set invalid points to NaN. PCL will ignore them if is_dense is false.
                cloud->points[index].x = cloud->points[index].y = cloud->points[index].z = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }
    return cloud;
}