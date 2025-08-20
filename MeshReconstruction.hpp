//
// Created by Hamza El-Kebir on 8/11/25.
//

#ifndef VIPER_MESHRECONSTRUCTION_HPP
#define VIPER_MESHRECONSTRUCTION_HPP

#include <open3D/Open3D.h>
#include "open3d/t/geometry/RaycastingScene.h"
#include "open3d/t/geometry/TriangleMesh.h"
#include "open3d/core/CUDAUtils.h" // For checking CUDA availability

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <filesystem> // For temporary file handling
#include <fstream>    // For file reading
#include <cstddef>    // For std::byte
#include <stdexcept>  // For std::runtime_error
#include <ctime>      // For unique filename generation

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

    /// Enum to specify the desired output 3D file format.
    enum class FileFormat3D {
        GLB,  /// GL Transmission Format Binary
        GLTF, /// GL Transmission Format (JSON)
        OBJ,  /// Wavefront OBJ
        DAE,  /// Collada
        STL   /// Stereolithography
    };

    /**
 * @brief Serializes an Open3D TriangleMesh into an in-memory byte buffer.
 *
 * This function writes the mesh to a temporary file and reads it back into a
 * buffer, as Open3D's I/O operations are file-based. The temporary file is
 * deleted immediately after being read.
 *
 * @param mesh A shared_ptr to the open3d::geometry::TriangleMesh to serialize.
 * @param format The target file format specified by the FileFormat3D enum.
 * @return A shared_ptr to a std::vector<std::byte> containing the serialized
 * mesh data. Returns nullptr on failure.
 */
    std::shared_ptr<std::vector<std::byte>> SerializeMesh(
            std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
            FileFormat3D format);

/**
 * @brief Performs ray picking/casting against a TriangleMesh on a specified device.
 *
 * This function projects a ray and finds the intersection point with the mesh.
 * It can automatically use the GPU if available, or a specific device can be provided.
 *
 * @tparam T The floating-point type of the ray vectors (e.g., float, double).
 * @param mesh A shared_ptr to the open3d::geometry::TriangleMesh to test against.
 * @param origin The starting point of the ray.
 * @param dir The unit direction vector of the ray.
 * @param device An optional open3d::core::Device to run the computation on.
 * If not provided, it will auto-detect and use CUDA if available, else CPU.
 * @return An std::optional<Eigen::Vector3<T>> containing the intersection
 * point if a hit occurs, otherwise std::nullopt.
 */
    template <typename T>
    std::optional<Eigen::Vector3<T>> RayPick(
            std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
            const Eigen::Vector3<T>& origin,
            const Eigen::Vector3<T>& dir,
            std::optional<open3d::core::Device> device = std::nullopt);
}


#endif //VIPER_MESHRECONSTRUCTION_HPP
