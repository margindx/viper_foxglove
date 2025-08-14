//
// Created by Hamza El-Kebir on 8/11/25.
//

#include "MeshReconstruction.hpp"

std::vector<Eigen::Vector3d> mdx::geometry::MatrixToVector3dVector(const Eigen::MatrixXd &matrix) {
    std::vector<Eigen::Vector3d> vector;
    vector.reserve(matrix.rows());
    for (int i = 0; i < matrix.rows(); ++i) {
        vector.emplace_back(matrix.row(i));
    }
    return vector;
}

std::shared_ptr<open3d::geometry::PointCloud>
mdx::geometry::PreparePointCloudForReconstruction(const Eigen::MatrixXd &points, const Eigen::MatrixXd &normals,
                                                  double voxel_size) {

    // Create a new point cloud object.
    auto pcd = std::make_shared<open3d::geometry::PointCloud>();
    pcd->points_ = MatrixToVector3dVector(points);
    pcd->normals_ = MatrixToVector3dVector(normals);

    // Apply voxel downsampling to reduce the number of points for reconstruction.
    // This helps manage memory and computation time for large point clouds.
    auto downsampled_pcd = pcd->VoxelDownSample(voxel_size);

    // Re-estimate normals after downsampling using a KDTree search.
    // This ensures consistent and accurate normals for the reconstruction.
    downsampled_pcd->EstimateNormals(open3d::geometry::KDTreeSearchParamKNN(50));

    return downsampled_pcd;
}

std::shared_ptr<open3d::geometry::PointCloud>
mdx::geometry::RefineNormals(std::shared_ptr<open3d::geometry::PointCloud> pcd, int k) {

    // Estimate normals based on the given number of nearest neighbors.
    pcd->EstimateNormals(open3d::geometry::KDTreeSearchParamKNN(k));

    // Orient normals consistently based on the tangent plane.
    // This is a crucial step for algorithms like Poisson reconstruction.
    pcd->OrientNormalsConsistentTangentPlane(k);

    return pcd;
}

std::pair<std::shared_ptr<open3d::geometry::TriangleMesh>, std::vector<double>>
mdx::geometry::PoissonReconstruction(std::shared_ptr<open3d::geometry::PointCloud> pcd, int depth) {

    // Perform Poisson reconstruction, returning the mesh and vertex densities.
    return open3d::geometry::TriangleMesh::CreateFromPointCloudPoisson(*pcd, depth);
}

std::shared_ptr<open3d::geometry::TriangleMesh>
mdx::geometry::FilterLowDensityVertices(std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
                                        const std::vector<double> &densities, double density_threshold) {

    std::vector<size_t> high_density_indices;
    // Iterate through the densities and collect indices of vertices with high density.
    for (size_t i = 0; i < densities.size(); ++i) {
        if (densities[i] > density_threshold) {
            high_density_indices.push_back(i);
        }
    }

    // Select the vertices with high density to create a new mesh.
    return mesh->SelectByIndex(high_density_indices);
}

std::shared_ptr<open3d::geometry::TriangleMesh>
mdx::geometry::CreateMesh(const Eigen::MatrixXd &points, const Eigen::MatrixXd &normals) {

    // 1. Prepare the point cloud (downsample and re-estimate normals).
    auto downsampled_pcd = PreparePointCloudForReconstruction(points, normals, 0.0008);

    // 2. Refine the normals for the downsampled point cloud.
    auto refined_pcd = RefineNormals(downsampled_pcd, 100);

    // 3. Perform Poisson surface reconstruction.
    auto result = PoissonReconstruction(refined_pcd, 8);
    auto mesh = result.first;
    auto densities = result.second;

    // 4. Filter the reconstructed mesh based on vertex density to remove artifacts.
    return FilterLowDensityVertices(mesh, densities, 0.01);
}
