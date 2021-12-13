#ifndef MESHDIFFERENCE_HPP
#define MESHDIFFERENCE_HPP

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

struct PointDistanceData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d point;
  double distance;

  PointDistanceData(const Eigen::Vector3d point, const double distance)
      : point(point), distance(distance){};
};

/**
 * @brief      Class for computing the difference between two meshes. Heatmap PC
 * can be exported as a .ply.
 */
class MeshDifference {
public:
  typedef std::vector<PointDistanceData,
                      Eigen::aligned_allocator<PointDistanceData>>
      DistanceDataVector;

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  samplingDensity  The sampling density of the source Mesh. units
   * in ~ points/m^2
   */
  MeshDifference(const double samplingDensity)
      : samplingDensity_(samplingDensity){};

  /**
   * @brief      Sets the source mesh.
   *
   * @param[in]  sourceMesh  The source mesh
   */
  void setSourceMesh(const pcl::PolygonMesh &sourceMesh) {
    // Set the source mesh
    sourceMesh_ = sourceMesh;

    // Reset the previously computed difference data
    meshDifference_.clear();
  };

  /**
   * @brief      Sets the target mesh.
   *
   * @param[in]  targetMesh  The target mesh
   */
  void setTargetMesh(const pcl::PolygonMesh &targetMesh) {
    // Set the target mesh
    targetMesh_ = targetMesh;

    // Reset the previously computed difference data
    meshDifference_.clear();
  };

  /**
   * @brief      Calculates the difference between the source and the target
   * mesh.
   */
  void computeDifference();

  /**
   * @brief      Saves the difference as a heatmap PC.
   *
   * @param[in]  filename     The output filename
   * @param[in]  minDistance  The minimum distance
   * @param[in]  maxDistance  The maximum distance
   * @param[in]  colormap     The colormap
   */
  void saveHeatmapMesh(const std::string &filename, const double minDistance,
                       const double maxDistance, const Colormap &colormap);

private:
  /**
   * @brief      Samples a surface mesh and return the sampled Pointcloud
   *
   * @param[in]  mesh             The mesh to be sampled
   * @param[in]  samplingDensity  The sampling density. Unit in [samples/m^2]
   *
   * @return     The sampled pointcloud
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr
  sampleSurfaceMesh(const pcl::PolygonMesh &mesh, const double samplingDensity);

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
  void sampleTriangleSurface(
      const Eigen::Vector3d &v0, const Eigen::Vector3d &v1,
      const Eigen::Vector3d &v2, const std::default_random_engine &generator,
      const std::uniform_real_distribution<double> &distribution,
      Eigen::Vector3d &sample);

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
  void sampleTriangleSurface(
      const Eigen::Vector3d p0, const Eigen::Vector3d p1,
      const Eigen::Vector3d p2, std::default_random_engine &generator,
      std::uniform_real_distribution<double> &distribution, Eigen::Vector3d &p);

  const double samplingDensity_; ///< Sampling density in points per m^2
  DistanceDataVector
      meshDifference_; ///< PC containing the distance Info between the sampled
                       ///< source Mesh and the target mesh
  pcl::PolygonMesh sourceMesh_; ///< Source Mesh
  pcl::PolygonMesh targetMesh_; ///< Target Mesh
};

#endif