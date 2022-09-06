#include <MeshDifference.hpp>
#include <algorithm>
#include <map>
#include <mesh_properties.hpp>
#include <options.hpp>
#include <pcl/common/centroid.h>
#include <set>

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



bool objectFilename(const std::filesystem::path& path)
{
    return path.filename().string().find("object") != std::string::npos;
}



int main(int argc, char** argv)
{
    // Parse input arguments
    const Options options = parse_options(argc, argv);

    // Hardcoded parameters. ToDo -> Consider adding these in the Options
    const double inlierThreshold = 0.05;
    const double samplingDensity = 0.0; // number of points per m^2
    constexpr int desired_scale = 0;
    constexpr float desired_dist = 1.0f;

    // Iterate source Path
    MeshData::Vector sourceDataVec;
    for (const auto& entry : std::filesystem::directory_iterator(options.source_mesh_path)) {
        if (validFilename(entry.path()) && objectFilename(entry.path())) {
            sourceDataVec.emplace_back(entry.path());
        }
    }
    std::sort(sourceDataVec.begin(), sourceDataVec.end());
    if (options.verbose) {
        std::cout << "Loaded " << sourceDataVec.size() << " source meshes\n";
    }
    if (options.verbose > 1) {
        for (const auto& m : sourceDataVec) {
            std::cout << "  " << m.filename.string() << "\n";
        }
    }

    // Iterate target Path
    MeshData::Vector targetDataVec;
    for (const auto& entry : std::filesystem::directory_iterator(options.target_mesh_path)) {
        if (validFilename(entry.path())) {
            targetDataVec.emplace_back(entry.path());
        }
    }
    std::sort(targetDataVec.begin(), targetDataVec.end());
    if (options.verbose) {
        std::cout << "Loaded " << targetDataVec.size() << " target meshes\n";
    }
    if (options.verbose > 1) {
        for (const auto& m : targetDataVec) {
            std::cout << "  " << m.filename.string() << "\n";
        }
    }

    // Store the TSV data so that they can be printed in order when running with multiple threads.
    std::vector<std::string> tsv_data(sourceDataVec.size() + 1);
    tsv_data[0] =
        "Source object\tTarget object\tAccuracy (m)\tCompleteness (%)\tDesired scale (%)\tMean observed dist (m)\tDesired observed dist (%)\n";

    // Create a map from target to source meshes.
    std::multimap<std::string, std::string> matches;
    std::set<std::string> matchedTargets;
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

#pragma omp critical(matches)
        {
            matches.insert({targetMeshDataIt->name, sourceMeshData.filename.string()});
            matchedTargets.insert(targetMeshDataIt->name);
        }

        // Compute the Mesh diff
        MeshDifference meshDifference(samplingDensity);
        meshDifference.setSourceMesh(sourceMeshData.mesh);
        meshDifference.setTargetMesh(targetMeshDataIt->mesh);
        const float accuracy = meshDifference.computeDifference(inlierThreshold);
        const float completeness = 100.0f * meshDifference.computeCompleteness(inlierThreshold);
        const float pc_desired_scale = percentage_at_scale(
            extract_mesh_scales(sourceMeshData.filename.string()), desired_scale);
        const float mean_dist = mean(extract_mesh_distances(sourceMeshData.filename.string()));
        const float pc_desired_dist = percentage_at_distance(
            extract_mesh_distances(sourceMeshData.filename.string()), desired_dist);

        // Save accuracy and completeness Heatmaps as .ply
        if (!options.heatmap_dir.empty()) {
            const std::string accuracyHeatmapFilename = options.heatmap_dir.string() + "/"
                + std::string(sourceMeshData.name.begin(), sourceMeshData.name.end() - 4)
                + "_acc_heatmap.ply";

            const std::string completenessHeatmapFilename = options.heatmap_dir.string() + "/"
                + std::string(targetMeshDataIt->name.begin(), targetMeshDataIt->name.end() - 4)
                + "_compl_heatmap.ply";

            Colormap colormap = Colormap::Turbo;

            std::filesystem::create_directories(options.heatmap_dir);

            // Save accuracy heatmap
            meshDifference.saveAccuracyHeatmap(
                accuracyHeatmapFilename, 0.0, inlierThreshold, colormap);

            // Save completeness heatmap
            meshDifference.saveCompletenessHeatmap(completenessHeatmapFilename,
                                                   0.0,
                                                   inlierThreshold,
                                                   tinycolormap::GetColor(0.0, colormap),
                                                   tinycolormap::GetColor(1.0, colormap));
        }

        // Show the results as TSV.
        tsv_data[i + 1] = sourceMeshData.filename.string() + "\t"
            + targetMeshDataIt->filename.string() + "\t" + std::to_string(accuracy) + "\t"
            + std::to_string(completeness) + "\t" + std::to_string(pc_desired_scale) + "\t"
            + std::to_string(mean_dist) + "\t" + std::to_string(pc_desired_dist) + "\n";
    }

    // Show matching information and duplicate matches.
    std::cout << "Matched " << matchedTargets.size() << "/" << targetDataVec.size()
              << " target meshes\n";
    for (const auto& m : targetDataVec) {
        const size_t n = matches.count(m.name);
        if (n == 0) {
            std::cout << m.filename.string() << " not matched by any object\n";
        }
        else if (n > 1) {
            std::cout << m.filename.string() << " matched by multiple objects:\n";
            const auto r = matches.equal_range(m.name);
            for (auto it = r.first; it != r.second; ++it) {
                std::cout << "  " << it->second << "\n";
            }
        }
    }

    // Save the TSV data.
    std::ofstream f(options.tsv_file);
    for (const auto& s : tsv_data) {
        f << s;
    }

    return 0;
}
