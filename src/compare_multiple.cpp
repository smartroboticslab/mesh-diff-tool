#include <MeshDifference.hpp>
#include <algorithm>
#include <filesystem>
#include <pcl/common/centroid.h>

class MeshData {
    public:
    typedef std::vector<MeshData, Eigen::aligned_allocator<MeshData>> Vector;

    std::filesystem::path filename;
    std::string name;
    pcl::PolygonMesh mesh;
    Eigen::Vector3d centroid;

    MeshData(const std::filesystem::path& path) : filename(path), name(path.filename().string())
    {
        pcl::io::loadPolygonFilePLY(filename.string(), mesh);
        computeCentroid();
    };

    bool operator<(const MeshData& rhs) const
    {
        return name < rhs.name;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW



    private:
    void computeCentroid()
    {
        // Use the built in PCL, PC generation
        Eigen::Vector4d centroid_h;
        pcl::PointCloud<pcl::PointXYZ>::Ptr tempPC(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromPCLPointCloud2(mesh.cloud, *tempPC);
        pcl::compute3DCentroid(*tempPC, centroid_h);
        centroid = centroid_h.head<3>();
    }
};



bool validFilename(const std::filesystem::path& path)
{
    const bool is_file = !std::filesystem::is_directory(path);
    const bool is_ply = path.extension() == ".ply";
    const std::string s = path;
    const std::string suffix = "_heatmap.ply";
    const bool is_heatmap = s.substr(s.size() - suffix.size()) == suffix;
    return is_file && is_ply && !is_heatmap;
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
    MeshData::Vector sourceDataVec;
    for (const auto& entry : std::filesystem::directory_iterator(sourcePath)) {
        if (validFilename(entry.path())) {
            sourceDataVec.emplace_back(entry.path());
        }
    }
    std::sort(sourceDataVec.begin(), sourceDataVec.end());

    // Iterate target Path
    MeshData::Vector targetDataVec;
    for (const auto& entry : std::filesystem::directory_iterator(targetPath)) {
        if (validFilename(entry.path())) {
            targetDataVec.emplace_back(entry.path());
        }
    }
    std::sort(targetDataVec.begin(), targetDataVec.end());

    // Store the TSV data so that they can be printed in order when running with multiple threads.
    std::vector<std::string> tsv_data(sourceDataVec.size() + 1);
    tsv_data[0] = "Detected object\tGround truth object\tAccuracy (m)\tCompleteness (%)\n";

    // Pairs to be compared.
    std::set<std::string> lookup;
#pragma omp parallel for
    for (size_t i = 0; i < sourceDataVec.size(); i++) {
        const auto& sourceMeshData = sourceDataVec[i];
        // Locate the target Mesh based on the centroid distance.
        const auto targetMeshDataIt =
            std::min_element(targetDataVec.begin(),
                             targetDataVec.end(),
                             [&sourceMeshData](const auto& lhs, const auto& rhs) {
                                 return (lhs.centroid - sourceMeshData.centroid).norm()
                                     < (rhs.centroid - sourceMeshData.centroid).norm();
                             });

#pragma omp critical(lookup)
        {
            // Check for matches to the same Mesh
            if (lookup.count(targetMeshDataIt->name)) {
                std::cerr << "Warning: duplicate object match to " << targetMeshDataIt->name
                          << "\n";
            }
            else {
                lookup.insert(targetMeshDataIt->name);
            }
        }

        // Compute the Mesh diff
        MeshDifference meshDifference(1000000.0);
        meshDifference.setSourceMesh(sourceMeshData.mesh);
        meshDifference.setTargetMesh(targetMeshDataIt->mesh);
        const float accuracy = meshDifference.computeDifference();

        const float completness = 0.0f;

        // Show the results as TSV.
        tsv_data[i + 1] = sourceMeshData.filename.string() + "\t"
            + targetMeshDataIt->filename.string() + "\t" + std::to_string(accuracy) + "\t"
            + std::to_string(completness) + "\n";

        // Save PC heatmap as a .ply
        const std::string heatmapFilename = outputPath + "/"
            + std::string(sourceMeshData.name.begin(), sourceMeshData.name.end() - 4)
            + "_heatmap.ply";

        const double minDistance = 0.0;
        const double maxDistance = 0.05;
        Colormap colormap = Colormap::Turbo;

        meshDifference.saveHeatmapMesh(heatmapFilename, minDistance, maxDistance, colormap);
    }

    for (const auto& s : tsv_data) {
        std::cout << s;
    }

    return 0;
}
