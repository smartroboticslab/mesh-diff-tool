#include <MeshDifference.hpp>
#include <pcl/conversions.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr MeshDifference::sampleSurfaceMesh(const pcl::PolygonMesh& mesh,
                                                                      const double samplingDensity)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr sampledPCPtr(new pcl::PointCloud<pcl::PointXYZ>());

    // Convert the Input mesh to VTK.
    vtkSmartPointer<vtkPolyData> polygonDataPtr = vtkSmartPointer<vtkPolyData>::New();
    pcl::io::mesh2vtk(mesh, polygonDataPtr);

    // Compute total area.
    const double area = computeTotalArea(polygonDataPtr);

    // Iterate each triangle and sample points
    if (polygonDataPtr->NeedToBuildCells())
        polygonDataPtr->BuildCells();

    // Iterate all the triangles and accumulate the surface area.
    vtkSmartPointer<vtkCellArray> cellsPtr = polygonDataPtr->GetPolys();

    const size_t cellSize = static_cast<size_t>(cellsPtr->GetNumberOfCells());

    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.0, 1.0);

    // Loop through cells
    for (size_t i = 0; i < cellSize; ++i) {
        // Get Cell and ignore if not triangle
        vtkCell* cell = polygonDataPtr->GetCell(i);
        if (cell) {
            if (cell->GetCellType() != VTK_TRIANGLE) {
                continue; // Skip non-triangle cells
            }

            // Convert to a triangle and compute area
            vtkTriangle* triangle = dynamic_cast<vtkTriangle*>(cell);
            const double area = triangle->ComputeArea();

            // Compute Number of points to be sampled.
            const size_t samples =
                static_cast<size_t>(std::max(1.0, round(area * samplingDensity)));
            // Extract triangle vertices.
            Eigen::Vector3d p0, p1, p2, p;
            triangle->GetPoints()->GetPoint(0, p0.data());
            triangle->GetPoints()->GetPoint(1, p1.data());
            triangle->GetPoints()->GetPoint(2, p2.data());

            for (size_t j = 0; j < samples; j++) {
                // Sample point on triangle surface
                sampleTriangleSurface(p0, p1, p2, generator, distribution, p);

                // Append P to pointcloud.
                sampledPCPtr->push_back(pcl::PointXYZ(p(0), p(1), p(2)));
            }
        }
    }

    return sampledPCPtr;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MeshDifference::meshToPointCloud(const pcl::PolygonMesh& mesh)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(mesh.cloud, *cloudPtr);
    return cloudPtr;
}

float MeshDifference::computeDifference(const double inlierThreshold)
{
    // Compute the distance between the source and the target Mesh.
    if (!(computeMesh2MeshDistance(
            sourceMesh_, targetMesh_, samplingDensity_, source2TargetDistance_)))
        return NAN;

    // Compute the RMSE of the inliers
    double sum_squares = 0.0;
    size_t no_inliers = 0;
    for (const auto& d : source2TargetDistance_) {
        if (d.distance > inlierThreshold)
            continue;
        sum_squares += d.distance * d.distance;
        no_inliers++;
    }

    return no_inliers ? std::sqrt(sum_squares / no_inliers) : 0.f;
}


float MeshDifference::computeCompleteness(const double inlierThreshold)
{
    // Compute the distance between the target and the source Mesh. targetMesh_
    if (!(computeMesh2MeshDistance(
            targetMesh_, sourceMesh_, samplingDensity_, target2SourceDistance_)))
        return NAN;

    // Compute the completeness percentage
    return static_cast<float>(std::count_if(
               target2SourceDistance_.begin(),
               target2SourceDistance_.end(),
               [&inlierThreshold](auto point) { return point.distance <= inlierThreshold; }))
        / target2SourceDistance_.size();
}


bool MeshDifference::computeMesh2MeshDistance(const pcl::PolygonMesh& comparedMesh,
                                              const pcl::PolygonMesh& referenceMesh,
                                              const double samplingDensity,
                                              DistanceDataVector& meshDifference)
{
    // Make sure the compared and reference meshes are not empty.
    if (comparedMesh.polygons.empty() || referenceMesh.polygons.empty())
        return false;

    // Reset the mesh difference vector.
    meshDifference.clear();

    // Convert the reference Mesh to VTK polydata format and instantiate the Class
    // used for the minimum distance computation
    vtkSmartPointer<vtkPolyData> referenceMeshVTKPtr;
    pcl::io::mesh2vtk(referenceMesh, referenceMeshVTKPtr);

    vtkSmartPointer<vtkImplicitPolyDataDistance> distance2MeshPtr =
        vtkSmartPointer<vtkImplicitPolyDataDistance>::New();
    distance2MeshPtr->SetInput(referenceMeshVTKPtr);

    // Sample the compared Mesh and generate a PC.
    auto sampledPCPtr = (samplingDensity == 0.0 ? meshToPointCloud(comparedMesh)
                                                : sampleSurfaceMesh(comparedMesh, samplingDensity));

    // Iterate the sampledPC and compute the distance to the target Mesh.
    for (const auto& point : *sampledPCPtr) {
        // Convert to Eigen.
        const Eigen::Vector3d p(point.x, point.y, point.z);

        // Compute the distance to the Mesh. ToDo.
        double buffer[3]{p(0), p(1), p(2)};
        double minDistance = std::fabs(distance2MeshPtr->EvaluateFunction(buffer));

        // Push result
        meshDifference.push_back(PointDistanceData(p, minDistance));
    }

    return true;
}

double MeshDifference::computeTotalArea(const vtkSmartPointer<vtkPolyData> polygonDataPtr)
{
    double result = 0;
    if (polygonDataPtr->NeedToBuildCells())
        polygonDataPtr->BuildCells();

    // Iterate all the triangles and accumulate the surface area.
    vtkSmartPointer<vtkCellArray> cellsPtr = polygonDataPtr->GetPolys();

    const size_t cellSize = static_cast<size_t>(cellsPtr->GetNumberOfCells());
    // Loop through cells
    for (size_t i = 0; i < cellSize; ++i) {
        // Get Cell and ignore if not triangle
        vtkCell* cell = polygonDataPtr->GetCell(i);

        if (cell) {
            if (cell->GetCellType() != VTK_TRIANGLE) {
                continue; // Skip non-triangle cells
            }

            // Convert to a triangle and compute area
            vtkTriangle* triangle = dynamic_cast<vtkTriangle*>(cell);
            const double area = triangle->ComputeArea();
            result += area;
        }
    }
    return result;
}

void MeshDifference::sampleTriangleSurface(const Eigen::Vector3d p0,
                                           const Eigen::Vector3d p1,
                                           const Eigen::Vector3d p2,
                                           std::default_random_engine& generator,
                                           std::uniform_real_distribution<double>& distribution,
                                           Eigen::Vector3d& p)
{
    const double alpha = distribution(generator);
    const double beta = distribution(generator);
    p = p0 + alpha * (p1 - p0) + beta * (p2 - p1);
}

double MeshDifference::rmse() const
{
    double sum_squares = 0.0;
    for (const auto& d : source2TargetDistance_) {
        sum_squares += d.distance * d.distance;
    }
    return std::sqrt(sum_squares / source2TargetDistance_.size());
}

void MeshDifference::saveAccuracyHeatmap(const std::string& filename,
                                         const double minDistance,
                                         const double maxDistance,
                                         const Colormap& colormap)
{
    // Check if output cloud is empty
    if (source2TargetDistance_.empty())
        return;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr heatmapPCPtr(new pcl::PointCloud<pcl::PointXYZRGB>());

    for (const auto& heatmapEntry : source2TargetDistance_) {
        // Extract position.
        const Eigen::Vector3d position = heatmapEntry.point;

        // Extract distance.
        const double distance = heatmapEntry.distance;

        // Scale distance in the [0, 1] range.
        const double scaledDistance =
            std::max(std::min((distance - minDistance) / (maxDistance - minDistance), 1.0), 0.0);

        // Map to color and push to the pointcloud.
        const Color color = tinycolormap::GetColor(scaledDistance, colormap);

        pcl::PointXYZRGB colouredPoint;
        colouredPoint.x = position(0);
        colouredPoint.y = position(1);
        colouredPoint.z = position(2);
        colouredPoint.r = static_cast<uint8_t>(color.r() * 255.0);
        colouredPoint.g = static_cast<uint8_t>(color.g() * 255.0);
        colouredPoint.b = static_cast<uint8_t>(color.b() * 255.0);

        heatmapPCPtr->push_back(colouredPoint);
    }

    // Save to disk.
    const bool export_binary = true;
    pcl::io::savePLYFile(filename, *heatmapPCPtr, export_binary);
}

void MeshDifference::saveCompletenessHeatmap(const std::string& filename,
                                             const double minDistance,
                                             const double maxDistance,
                                             const Color& inlierColor,
                                             const Color& outlierColor)
{
    // Check if output cloud is empty
    if (target2SourceDistance_.empty())
        return;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr heatmapPCPtr(new pcl::PointCloud<pcl::PointXYZRGB>());

    for (const auto& heatmapEntry : target2SourceDistance_) {
        // Extract position.
        const Eigen::Vector3d position = heatmapEntry.point;

        // Extract distance.
        const double distance = heatmapEntry.distance;

        // Check if point is inlier or outlier
        const bool isInlier = (distance >= minDistance) && (distance <= maxDistance);

        // Map to color and push to the pointcloud.
        const tinycolormap::Color color = isInlier ? inlierColor : outlierColor;

        pcl::PointXYZRGB colouredPoint;
        colouredPoint.x = position(0);
        colouredPoint.y = position(1);
        colouredPoint.z = position(2);
        colouredPoint.r = static_cast<uint8_t>(color.r() * 255.0);
        colouredPoint.g = static_cast<uint8_t>(color.g() * 255.0);
        colouredPoint.b = static_cast<uint8_t>(color.b() * 255.0);

        heatmapPCPtr->push_back(colouredPoint);
    }

    // Save to disk.
    const bool export_binary = true;
    pcl::io::savePLYFile(filename, *heatmapPCPtr, export_binary);
}
