//
// Created by Hamza El-Kebir on 8/11/25.
//

#ifndef VIPER_MESHRECONSTRUCTION_HPP
#define VIPER_MESHRECONSTRUCTION_HPP

#include <Open3D/Open3D.h>
#include <iostream>
#include <memory>
#include <string>

namespace mdx::geometry {

/// @brief Converts an Eigen::MatrixXd to a std::vector<Eigen::Vector3d>.
/// This function is necessary because the Open3D C++ API uses std::vector<Eigen::Vector3d>
/// for point cloud data, while the Python API often works with numpy arrays (which
/// correspond to Eigen::MatrixXd in the C++ binding).
/// @param matrix The input Eigen::MatrixXd.
/// @return A std::vector<Eigen::Vector3d> with the converted data.
    std::vector<Eigen::Vector3d> MatrixToVector3dVector(const Eigen::MatrixXd &matrix);

/// @brief Prepares a point cloud for reconstruction by downsampling and re-estimating normals.
/// The original Python function was named `densify_point_cloud` but performed downsampling.
/// This C++ function accurately reflects that a downsampling step is often used to
/// prepare point clouds for surface reconstruction.
/// @param points A Eigen::MatrixXd containing the 3D points.
/// @param normals A Eigen::MatrixXd containing the normal vectors.
/// @param voxel_size The size of the voxel grid for downsampling.
/// @return A shared pointer to the prepared PointCloud.
    std::shared_ptr<open3d::geometry::PointCloud> PreparePointCloudForReconstruction(
            const Eigen::MatrixXd &points,
            const Eigen::MatrixXd &normals,
            double voxel_size = 0.005);

/// @brief Improves normal consistency to assist in surface reconstruction.
/// @param pcd A shared pointer to the input PointCloud.
/// @param k The number of nearest neighbors to use for normal estimation.
/// @return A shared pointer to the PointCloud with refined normals.
    std::shared_ptr<open3d::geometry::PointCloud> RefineNormals(
            std::shared_ptr<open3d::geometry::PointCloud> pcd,
            int k = 100);

/// @brief Reconstructs a surface mesh using Poisson surface reconstruction.
/// @param pcd A shared pointer to the input PointCloud.
/// @param depth The octree depth, which controls the level of detail.
/// A higher value results in a finer mesh.
/// @return A pair containing a shared pointer to the reconstructed TriangleMesh and the vertex densities.
    std::pair<std::shared_ptr<open3d::geometry::TriangleMesh>, std::vector<double>> PoissonReconstruction(
            std::shared_ptr<open3d::geometry::PointCloud> pcd,
            int depth = 8);

/// @brief Removes floating artifacts from the Poisson surface by filtering low-density areas.
/// @param mesh A shared pointer to the reconstructed mesh.
/// @param densities A vector of doubles representing the density of each vertex.
/// @param density_threshold The threshold for removing low-density vertices.
/// @return A shared pointer to the cleaned-up TriangleMesh.
    std::shared_ptr<open3d::geometry::TriangleMesh> FilterLowDensityVertices(
            std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
            const std::vector<double> &densities,
            double density_threshold = 0.01);

/// @brief The main function that orchestrates the entire reconstruction process.
/// This is the C++ equivalent of the `create_mesh` function in your Python script.
/// @param points A Eigen::MatrixXd containing the 3D points.
/// @param normals A Eigen::MatrixXd containing the normal vectors.
/// @return A shared pointer to the final reconstructed and filtered TriangleMesh.
    std::shared_ptr<open3d::geometry::TriangleMesh> CreateMesh(
            const Eigen::MatrixXd &points,
            const Eigen::MatrixXd &normals);
}


#endif //VIPER_MESHRECONSTRUCTION_HPP
