#ifndef source2TargetDistance_HPP
#define source2TargetDistance_HPP

#include <iostream>
#include <pcl/io/auto_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <random>
#include <tinycolormap.hpp>
#include <vtkImplicitPolyDataDistance.h>
#include <vtkTriangle.h>

typedef tinycolormap::ColormapType Colormap;
typedef tinycolormap::Color Color;

struct PointDistanceData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d point;
    double distance;

    PointDistanceData(const Eigen::Vector3d point, const double distance) :
            point(point), distance(distance){};
};

/**
 * @brief      Class for computing the difference between two meshes. Heatmap PC
 * can be exported as a .ply.
 */
class MeshDifference {
    public:
    typedef std::vector<PointDistanceData, Eigen::aligned_allocator<PointDistanceData>>
        DistanceDataVector;

    /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  samplingDensity  The sampling density of the source Mesh. units
   * in ~ points/m^2
   */
    MeshDifference(const double samplingDensity) : samplingDensity_(samplingDensity){};

    /**
   * @brief      Sets the source mesh.
   *
   * @param[in]  sourceMesh  The source mesh
   */
    void setSourceMesh(const pcl::PolygonMesh& sourceMesh)
    {
        // Set the source mesh
        sourceMesh_ = sourceMesh;

        // Reset the previously computed difference data
        source2TargetDistance_.clear();
    };

    /**
   * @brief      Sets the target mesh.
   *
   * @param[in]  targetMesh  The target mesh
   */
    void setTargetMesh(const pcl::PolygonMesh& targetMesh)
    {
        // Set the target mesh
        targetMesh_ = targetMesh;

        // Reset the previously computed difference data
        source2TargetDistance_.clear();
    };

    /**
     * @brief      Calculates the difference between the target and the source
     *             and returns the completeness percentage
     *
     * @param[in]  inlierThreshold  The threshold in meters used to clasify a
     *                              point as inlier (i.e. distance <=
     *                              inlierThreshold). The input value only
     *                              affects the computation of the completeness
     *                              percentage.
     *
     * @return     The completeness percentage.
     */
    float computeCompleteness(const double inlierThreshold);

    /**
     * @brief      Calculates the difference between the source and the target
     *             mesh and returns the RMSE error.
     *
     * @param[in]  inlierThreshold  The threshold in meters used to clasify a
     *                              point as inlier (i.e. distance <=
     *                              inlierThreshold). The input value only
     *                              affects the computation of the RMSE.
     *
     * @return     The RMSE error in metres.
     */
    float computeDifference(const double inlierThreshold);


    /**
     * @brief      Calculates the difference between two meshes. The surface of
     *             the comparedMesh is sampled and the distance between each
     *             sampled point and the closest triangle of the referenceMesh
     *             is computed.
     *
     * @param[in]  comparedMesh     The compared mesh
     * @param[in]  referenceMesh    The reference mesh i.e. the distances will
     *                              be computed relatively to its triangles.
     * @param[in]  samplingDensity  The sampling density used for the spatial mesh sampling. Units in  #points/m^2
     * @param      meshDifference   The PC containing the distance info.
     *
     * @return     True when successful.
     */
    bool computeMesh2MeshDistance(const pcl::PolygonMesh& comparedMesh,
                                  const pcl::PolygonMesh& referenceMesh,
                                  const double samplingDensity,
                                  DistanceDataVector& meshDifference);

    /**
     * @brief      Saves the completeness heatmap. Points with distance d
     *             satisfying the condition: (d>= minDistance && d<=
     *             maxDistance) are considered inliers and plotted using
     *             inlierColor. Otherwise they are considered outliers and
     *             plotted using outlierColor.
     *
     * @param[in]  filename      The output filename
     * @param[in]  minDistance   The minimum distance used in the computation of
     *                           inliers
     * @param[in]  maxDistance   The maximum distance used in the computation of
     *                           inliers
     * @param[in]  inlierColor   The inlier color
     * @param[in]  outlierColor  The outlier color
     */
    void saveCompletenessHeatmap(const std::string& filename,
                                 const double minDistance,
                                 const double maxDistance,
                                 const Color& inlierColor,
                                 const Color& outlierColor);

    /**
     * @brief      Saves the difference as a heatmap PC.
     *
     * @param[in]  filename     The output filename
     * @param[in]  minDistance  The minimum distance
     * @param[in]  maxDistance  The maximum distance
     * @param[in]  colormap     The colormap
     */
    void saveAccuracyHeatmap(const std::string& filename,
                             const double minDistance,
                             const double maxDistance,
                             const Colormap& colormap);

    private:
    /**
   * @brief      Samples a surface mesh and return the sampled Pointcloud
   *
   * @param[in]  mesh             The mesh to be sampled
   * @param[in]  samplingDensity  The sampling density. Unit in [samples/m^2]
   *
   * @return     The sampled pointcloud
   */
    pcl::PointCloud<pcl::PointXYZ>::Ptr sampleSurfaceMesh(const pcl::PolygonMesh& mesh,
                                                          const double samplingDensity);

    /**
   * @brief      Samples a point on the triangle surface given by the vertices
   * v0, v1, v2
   *
   * @param[in]  v0            Triangle vertex 0
   * @param[in]  v1            Triangle vertex 1
   * @param[in]  v2            Triangle vertex 2
   * @param      generator     The random generator required for the sampling
   * @param      distribution  Uniform distribution required for the sampling
   * @param      sample        The sampled point
   */
    void sampleTriangleSurface(const Eigen::Vector3d& v0,
                               const Eigen::Vector3d& v1,
                               const Eigen::Vector3d& v2,
                               const std::default_random_engine& generator,
                               const std::uniform_real_distribution<double>& distribution,
                               Eigen::Vector3d& sample);

    /**
   * @brief      Calculates the total surface area of the input Mesh
   *
   * @param[in]  polygonDataPtr  Ptr to the input mesh
   *
   * @return     The total surface area.
   */
    double computeTotalArea(const vtkSmartPointer<vtkPolyData> polygonDataPtr);

    /**
   * @brief      Samples a point from a 3D triangular surface with vertices (p0,
   * p1, p2).
   *
   * @param[in]  p0            3D position of vertex 0
   * @param[in]  p1            3D position of vertex 1
   * @param[in]  p2            3D position of vertex 2
   * @param      generator     Random engine generator
   * @param      distribution  Uniform distribution
   * @param      p             The 3D sampled pooint
   */
    void sampleTriangleSurface(const Eigen::Vector3d p0,
                               const Eigen::Vector3d p1,
                               const Eigen::Vector3d p2,
                               std::default_random_engine& generator,
                               std::uniform_real_distribution<double>& distribution,
                               Eigen::Vector3d& p);

    double rmse() const;

    const double samplingDensity_; ///< Sampling density in points per m^2
    DistanceDataVector
        source2TargetDistance_; ///< PC containing the distance Info between the sampled source Mesh and the target mesh. Used for visualising the accuracy of the source Mesh
    DistanceDataVector
        target2SourceDistance_; ///< PC containing the distance Info between the sampled target Mesh and the source mesh. Used for visualising the completeness of the reconstruction
    pcl::PolygonMesh sourceMesh_; ///< Source Mesh
    pcl::PolygonMesh targetMesh_; ///< Target Mesh
};

#endif
