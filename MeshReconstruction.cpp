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
    auto [mesh, densities] = open3d::geometry::TriangleMesh::CreateFromPointCloudPoisson(*pcd, depth);
    return {mesh, densities};
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

std::shared_ptr<std::vector<std::byte>>
mdx::geometry::SerializeMesh(std::shared_ptr<open3d::geometry::TriangleMesh> mesh, mdx::geometry::FileFormat3D format) {
    // 1. --- Input Validation ---
    if (!mesh) {
        open3d::utility::LogWarning("Input mesh is null.");
        return nullptr;
    }

    // 2. --- Determine File Extension and Create Temporary Path ---
    std::string extension;
    switch (format) {
        case FileFormat3D::GLB:  extension = ".glb"; break;
        case FileFormat3D::GLTF: extension = ".gltf"; break;
        case FileFormat3D::OBJ:  extension = ".obj"; break;
        case FileFormat3D::DAE:  extension = ".dae"; break;
        case FileFormat3D::STL:  extension = ".stl"; break;
        default:
            open3d::utility::LogError("Unsupported file format specified.");
            return nullptr;
    }

    // Create a unique temporary file path to avoid collisions.
    std::filesystem::path temp_path;
    try {
        auto temp_dir = std::filesystem::temp_directory_path();
        std::string unique_filename = "temp_mesh_" + std::to_string(std::time(nullptr)) + "_" + std::to_string(rand()) + extension;
        temp_path = temp_dir / unique_filename;
    } catch (const std::filesystem::filesystem_error& e) {
        open3d::utility::LogError("Failed to create temporary file path: {}", e.what());
        return nullptr;
    }


    // 3. --- Write Mesh to Temporary File ---
    // The WriteTriangleMesh function infers the format from the file extension.
    if (!open3d::io::WriteTriangleMesh(temp_path.string(), *mesh)) {
        open3d::utility::LogError("WriteTriangleMesh returned false for path: {}", temp_path.string());
        // Attempt to clean up even on failure
        std::error_code ec;
        std::filesystem::remove(temp_path, ec);
        return nullptr;
    }

    // 4. --- Read File Content into a Byte Buffer ---
    // Open the file in binary mode and position the cursor at the end.
    std::ifstream file(temp_path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        open3d::utility::LogError("Could not open temporary file for reading: {}", temp_path.string());
        std::error_code ec;
        std::filesystem::remove(temp_path, ec); // Clean up
        return nullptr;
    }

    // Get the file size and seek back to the beginning.
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    // Create a buffer of the appropriate size.
    auto buffer = std::make_shared<std::vector<std::byte>>(static_cast<size_t>(size));

    // Read the entire file into the buffer.
    if (!file.read(reinterpret_cast<char*>(buffer->data()), size)) {
        open3d::utility::LogError("Failed to read temporary file into buffer: {}", temp_path.string());
        file.close();
        std::error_code ec;
        std::filesystem::remove(temp_path, ec); // Clean up
        return nullptr;
    }

    // 5. --- Cleanup and Return ---
    file.close();
    std::error_code ec;
    if (!std::filesystem::remove(temp_path, ec)) {
        // Log if cleanup fails, but don't prevent returning the data.
        open3d::utility::LogWarning("Failed to remove temporary file '{}': {}", temp_path.string(), ec.message());
    }

    return buffer;
}

template<typename T>
std::optional<Eigen::Vector3<T>>
mdx::geometry::RayPick(std::shared_ptr<open3d::geometry::TriangleMesh> mesh, const Eigen::Vector3<T> &origin,
                       const Eigen::Vector3<T> &dir, std::optional<open3d::core::Device> device) {

    // 1. --- Determine computation device ---
    open3d::core::Device selected_device("CPU:0"); // Default to CPU
    if (device.has_value()) {
        selected_device = device.value();
        // Fallback if user-specified CUDA device is not actually available
        if (selected_device.GetType() == open3d::core::Device::DeviceType::CUDA && !open3d::core::cuda::IsAvailable()) {
            open3d::utility::LogWarning("User specified CUDA, but it is not available. Falling back to CPU.");
            selected_device = open3d::core::Device("CPU:0");
        }
    } else {
        // Auto-detect best available device if none is specified
        if (open3d::core::cuda::IsAvailable()) {
            selected_device = open3d::core::Device("CUDA:0");
        }
    }

    // 2. --- Convert legacy mesh to tensor-based mesh ---
    // First, create the tensor mesh on the CPU, specifying dtypes for vertices
    // (float/double) and triangles (int). This resolves potential API
    // version mismatches that cause the linter error.
    auto t_mesh_cpu = open3d::t::geometry::TriangleMesh::FromLegacy(
            *mesh, open3d::core::Float32, open3d::core::Int32);

    // Then, move the mesh to the selected device (CPU or CUDA).
    auto t_mesh = t_mesh_cpu.To(selected_device);

    // 3. --- Create a raycasting scene and add the mesh ---
    // The scene is implicitly created on the same device as the mesh.
    open3d::t::geometry::RaycastingScene scene;
    scene.AddTriangles(t_mesh);

    // 4. --- Create the ray tensor on the selected device ---
    open3d::core::Tensor rays(std::vector<float>{
            static_cast<float>(origin.x()), static_cast<float>(origin.y()), static_cast<float>(origin.z()),
            static_cast<float>(dir.x()), static_cast<float>(dir.y()), static_cast<float>(dir.z())
    }, {1, 6}, open3d::core::Float32, selected_device);

    // 5. --- Cast the ray ---
    auto result = scene.CastRays(rays);

    // 6. --- Process the result ---
    // The 't_hit' tensor contains the distance to the intersection.
    // A value of infinity means no hit.
    // Move the result to the CPU to access its value.
    float t_hit = result["t_hit"].To(open3d::core::Device("CPU:0")).ToFlatVector<float>()[0];

    if (std::isfinite(t_hit)) {
        // Calculate the intersection point: origin + t * direction
        Eigen::Vector3<T> intersection_point = origin + dir * static_cast<T>(t_hit);
        return intersection_point;
    }

    // No intersection found
    return std::nullopt;
}

template std::optional<Eigen::Vector3<double>> mdx::geometry::RayPick<double>(
        std::shared_ptr<open3d::geometry::TriangleMesh>,
        const Eigen::Vector3<double>&,
        const Eigen::Vector3<double>&,
        std::optional<open3d::core::Device>);

template std::optional<Eigen::Vector3<float>> mdx::geometry::RayPick<float>(
        std::shared_ptr<open3d::geometry::TriangleMesh>,
        const Eigen::Vector3<float>&,
        const Eigen::Vector3<float>&,
        std::optional<open3d::core::Device>);
