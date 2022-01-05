#include <MeshDifference.hpp>
#include <algorithm>
#include <filesystem>
#include <pcl/common/centroid.h>

struct MeshData {
    std::filesystem::path filename;
    std::string name;
    pcl::PolygonMesh mesh;
    Eigen::Vector3d centroid;

    MeshData(const std::filesystem::path& path) : filename(path), name(path.filename().string())
    {
        pcl::io::loadPolygonFilePLY(filename.string(), mesh);
        centroid = computeCentroid(mesh);
    };

    static Eigen::Vector3d computeCentroid(const pcl::PolygonMesh& mesh)
    {
        // Use the built in PCL, PC generation
        Eigen::Vector4d centroid_h;
        pcl::PointCloud<pcl::PointXYZ>::Ptr tempPC(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromPCLPointCloud2(mesh.cloud, *tempPC);
        pcl::compute3DCentroid(*tempPC, centroid_h);
        return centroid_h.head<3>();
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef std::vector<MeshData, Eigen::aligned_allocator<MeshData>> MeshDataVector;

bool operator<(const MeshData& lhs, const MeshData& rhs)
{
    return lhs.name < rhs.name;
}



bool validFilename(const std::filesystem::path& path)
{
    const std::string s = path;
    const std::string suffix = "_heatmap.ply";
    return path.extension() == ".ply" && s.substr(s.size() - suffix.size()) != suffix;
}



void usage(const char* program_name)
{
    std::cout << "Usage: " << program_name << " MESH_DIR GT_MESH_DIR [HEATMAP_DIR]\n";
}



int main(int argc, char** argv)
{
    if (argc != 3 && argc != 4) {
        usage(argv[0]);
        return 2;
    }
    const std::string sourcePath(argv[1]);
    const std::string targetPath(argv[2]);

    const std::string outputPath = (argc == 4) ? std::string(argv[3]) : sourcePath;

    // Iterate source Path
    MeshDataVector sourceDataVec;
    for (const auto& entry : std::filesystem::directory_iterator(sourcePath)) {
        if (validFilename(entry.path())) {
            sourceDataVec.emplace_back(entry.path());
        }
    }
    std::sort(sourceDataVec.begin(), sourceDataVec.end());

    // Iterate target Path
    MeshDataVector targetDataVec;
    for (const auto& entry : std::filesystem::directory_iterator(targetPath)) {
        if (validFilename(entry.path())) {
            targetDataVec.emplace_back(entry.path());
        }
    }
    std::sort(targetDataVec.begin(), targetDataVec.end());

    // Print the TSV header.
    std::cout << "Detected object\tGround truth object\tAccuracy (m)\tCompleteness (%)\n";
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
            std::cerr << "Warning: duplicate object match to " << targetMeshDataIt->name << "\n";
        }
        else {
            lookup.insert(targetMeshDataIt->name);
        }

        // Compute the Mesh diff
        meshDifference.setSourceMesh(sourceMeshData.mesh);
        meshDifference.setTargetMesh(targetMeshDataIt->mesh);
        meshDifference.computeDifference();

        const float accuracy = 0.0f;
        const float completness = 0.0f;

        // Show the results as TSV.
        std::cout << sourceMeshData.filename.string() << "\t" << targetMeshDataIt->filename.string()
                  << "\t" << accuracy << "\t" << completness << "\n";

        // Save PC heatmap as a .ply
        const std::string heatmapFilename = outputPath
            + std::string(sourceMeshData.name.begin(), sourceMeshData.name.end() - 4)
            + "_heatmap.ply";

        const double minDistance = 0.0;
        const double maxDistance = 0.05;
        Colormap colormap = Colormap::Turbo;

        meshDifference.saveHeatmapMesh(heatmapFilename, minDistance, maxDistance, colormap);
    }

    return 0;
}
