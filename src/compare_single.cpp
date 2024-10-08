/*
 * SPDX-FileCopyrightText: 2021-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021-2022 Dimos Tzoumanikas
 * SPDX-FileCopyrightText: 2022-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

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



int main(int argc, char** argv)
{
    // Parse input arguments
    const Options options = parse_options(argc, argv);

    // Hardcoded parameters. ToDo -> Consider adding these in the Options
    const double samplingDensity = 0.0; // number of points per m^2
    constexpr int desired_scale = 0;
    constexpr float desired_dist = 3.0f;

    // Load the meshes.
    pcl::PolygonMesh source_mesh;
    pcl::io::loadPolygonFilePLY(options.source_mesh_path.string(), source_mesh);
    pcl::PolygonMesh target_mesh;
    pcl::io::loadPolygonFilePLY(options.target_mesh_path.string(), target_mesh);

    // Compute the Mesh diff
    MeshDifference meshDifference(samplingDensity);
    meshDifference.setSourceMesh(source_mesh);
    meshDifference.setTargetMesh(target_mesh);
    const float accuracy = meshDifference.computeDifference(options.inlier_threshold);
    const float completeness =
        100.0f * meshDifference.computeCompleteness(options.inlier_threshold);
    const float pc_desired_scale =
        percentage_at_scale(extract_mesh_scales(options.source_mesh_path.string()), desired_scale);
    const float mean_dist = mean(extract_mesh_distances(options.source_mesh_path.string()));
    const float pc_desired_dist = percentage_at_distance(
        extract_mesh_distances(options.source_mesh_path.string()), desired_dist);

    // Save accuracy and completeness Heatmaps as .ply
    if (!options.heatmap_dir.empty()) {
        Colormap colormap = Colormap::Turbo;
        // std::filesystem::create_directories() fails on symbolic links to directories so test for
        // existence first.
        if (!std::filesystem::exists(options.heatmap_dir)) {
            std::filesystem::create_directories(options.heatmap_dir);
        }
        const std::string accuracyHeatmapFilename =
            options.heatmap_dir.string() + "/acc_heatmap.ply";
        meshDifference.saveAccuracyHeatmap(
            accuracyHeatmapFilename, 0.0, options.inlier_threshold, colormap);
        const std::string completenessHeatmapFilename =
            options.heatmap_dir.string() + "/compl_heatmap.ply";
        meshDifference.saveCompletenessHeatmap(completenessHeatmapFilename,
                                               0.0,
                                               options.inlier_threshold,
                                               tinycolormap::GetColor(0.0, colormap),
                                               tinycolormap::GetColor(1.0, colormap));
    }

    // Write the TSV data.
    {
        std::ofstream f(options.tsv_file);
        f << "Mesh\tReference mesh\tAccuracy (m)\tCompleteness (%)\tDesired scale (%)\tMean observed dist (m)\tDesired observed dist (%)\n";
        f << options.source_mesh_path.string() + "\t" + options.target_mesh_path.string() + "\t"
                + std::to_string(accuracy) + "\t" + std::to_string(completeness) + "\t"
                + std::to_string(pc_desired_scale) + "\t" + std::to_string(mean_dist) + "\t"
                + std::to_string(pc_desired_dist) + "\n";
    }
    return 0;
}
