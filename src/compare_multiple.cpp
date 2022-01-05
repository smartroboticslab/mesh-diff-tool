#include <MeshDifference.hpp>
#include <filesystem>
#include <pcl/common/centroid.h>

struct MeshData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::string name;
    pcl::PolygonMesh mesh;
    Eigen::Vector3d centroid;

    MeshData(const std::string& name,
             const pcl::PolygonMesh& mesh,
             const Eigen::Vector3d& centroid) :
            name(name), mesh(mesh), centroid(centroid){};
};

typedef std::vector<MeshData, Eigen::aligned_allocator<MeshData>> MeshDataVector;

Eigen::Vector3d computeCentroid(const pcl::PolygonMesh& mesh)
{
    // Use the built in PCL, PC generation
    Eigen::Vector4d centroid_h;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempPC(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::fromPCLPointCloud2(mesh.cloud, *tempPC);
    pcl::compute3DCentroid(*tempPC, centroid_h);
    return centroid_h.head<3>();
}

int main(int argc, char** argv)
{
    const std::string sourcePath(argv[1]);
    const std::string targetPath(argv[2]);

    const std::string outputPath = (argc == 4) ? std::string(argv[3]) : sourcePath;

    // Iterate source Path
    MeshDataVector sourceDataVec;
    for (const auto& entry : std::filesystem::directory_iterator(sourcePath)) {
        // Extract Mesh filename. ToDo -> check valid entries.
        const std::string meshPath = entry.path().string();
        const std::string meshFilename = entry.path().filename().string();
        pcl::PolygonMesh meshData;
        pcl::io::loadPolygonFilePLY(meshPath, meshData);

        // Compute centroid
        const Eigen::Vector3d meshCentroid = computeCentroid(meshData);

        // Combine in a single struct
        sourceDataVec.push_back(MeshData(meshFilename, meshData, meshCentroid));
    }

    // Iterate target Path
    MeshDataVector targetDataVec;
    for (const auto& entry : std::filesystem::directory_iterator(targetPath)) {
        // Extract Mesh filename. ToDo -> check valid entries.
        const std::string meshPath = entry.path().string();
        const std::string meshFilename = entry.path().filename().string();
        pcl::PolygonMesh meshData;
        pcl::io::loadPolygonFilePLY(meshPath, meshData);

        // Compute centroid
        const Eigen::Vector3d meshCentroid = computeCentroid(meshData);

        // Combine in a single struct
        targetDataVec.push_back(MeshData(meshFilename, meshData, meshCentroid));
    }

    // Pairs to be compared.
    std::set<std::string> lookup;
    MeshDifference meshDifference(1000000.0);

    for (const auto& sourceMeshData : sourceDataVec) {
        // Locate the target Mesh based on the centroid distance.
        const auto targetMeshDataIt =
            std::min_element(targetDataVec.begin(),
                             targetDataVec.end(),
                             [&sourceMeshData](const auto& lhs, const auto& rhs) {
                                 return (lhs.centroid - sourceMeshData.centroid).norm()
                                     < (rhs.centroid - sourceMeshData.centroid).norm();
                             });

        // Check for matches to the same Mesh
        if (lookup.count(targetMeshDataIt->name)) {
            // ToDo -> output a warning.
        }
        else {
            lookup.insert(targetMeshDataIt->name);
        }

        // Compute Euclidean distance between centroids
        const double centroidError = (sourceMeshData.centroid - targetMeshDataIt->centroid).norm();

        // Print Matches
        std::cout << "Source " << sourceMeshData.name << " matched to target "
                  << targetMeshDataIt->name << std::endl;

        // Compute the Mesh diff
        meshDifference.setSourceMesh(sourceMeshData.mesh);
        meshDifference.setTargetMesh(targetMeshDataIt->mesh);
        meshDifference.computeDifference();

        // Save PC heatmap as a .ply
        const std::string heatmapFilename = outputPath
            + std::string(sourceMeshData.name.begin(), sourceMeshData.name.end() - 4)
            + "_heatmap.ply";

        const double minDistance = 0.0;
        const double maxDistance = 0.05;
        Colormap colormap = Colormap::Turbo;

        meshDifference.saveHeatmapMesh(heatmapFilename, minDistance, maxDistance, colormap);
    }

    return 1;
}