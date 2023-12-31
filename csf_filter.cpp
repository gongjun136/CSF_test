/*
 * @Author: gongjun136 gongjun136@gmail.com
 * @Date: 2023-10-17 18:51:11
 * @LastEditors: gongjun136 gongjun136@gmail.com
 * @LastEditTime: 2023-10-17 19:14:16
 * @FilePath: /test/CSF_test/csf_filter.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <iostream>
#include <vector>
#include <fstream>
#include <CSF.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::cerr << "Usage: ./convert_pcd_to_txt <input.pcd> <output.pcd>" << std::endl;
        return -1;
    }

    // Load the PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1)
    {
        std::cerr << "Couldn't read the PCD file." << std::endl;
        return -1;
    }
    else
    {
        cloud->width = cloud->points.size();
        cloud->height = 1;
        std::cout << "load PCD file size: " << cloud->points.size();
    }

    // Convert PCL point cloud to CSF format
    std::vector<csf::Point> pointCloud;
    for (const auto &point : cloud->points)
    {
        csf::Point csfPoint;
        csfPoint.x = point.x;
        csfPoint.y = point.y;
        csfPoint.z = point.z;
        pointCloud.push_back(csfPoint);
    }
    // Start timing here
    auto start_time = std::chrono::high_resolution_clock::now();

    // Create and set up the CSF object
    CSF csf_filter;
    csf_filter.setPointCloud(pointCloud);
    // Set parameters (you might need to adjust these based on your dataset)
    csf_filter.params.bSloopSmooth = true;
    csf_filter.params.class_threshold = 2;

    // Perform the filtering
    std::vector<int> groundIndices, offGroundIndices;
    csf_filter.do_filtering(groundIndices, offGroundIndices);

    // Convert filtered points back to PCL format
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (int idx : groundIndices)
    {
        pcl::PointXYZ pclPoint;
        pclPoint.x = pointCloud[idx].x;
        pclPoint.y = pointCloud[idx].y;
        pclPoint.z = pointCloud[idx].z;
        filteredCloud->points.push_back(pclPoint);
    }
    filteredCloud->width = filteredCloud->points.size();
    filteredCloud->height = 1;

    // End timing here
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    std::cout << "Total execution time: " << duration << " ms" << std::endl;

    // Save the filtered point cloud to a new PCD file
    pcl::io::savePCDFileASCII(argv[2], *filteredCloud);
    std::cout << "Filtered point cloud saved to " << argv[2] << std::endl;

    return 0;
}
