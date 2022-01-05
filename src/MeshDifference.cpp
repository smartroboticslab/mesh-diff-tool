#include <MeshDifference.hpp>

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

void MeshDifference::computeDifference()
{
    // Make sure the source and target Meshes are not empty.
    if (sourceMesh_.polygons.empty() || targetMesh_.polygons.empty())
        return;

    // Reset the mesh difference vector.
    meshDifference_.clear();

    // Convert the target Mesh to VTK polydata format and instantiate the Class
    // used for the distance computation
    vtkSmartPointer<vtkPolyData> targetMeshVTKPtr;
    pcl::io::mesh2vtk(targetMesh_, targetMeshVTKPtr);

    vtkSmartPointer<vtkImplicitPolyDataDistance> distance2MeshPtr =
        vtkSmartPointer<vtkImplicitPolyDataDistance>::New();
    distance2MeshPtr->SetInput(targetMeshVTKPtr);

    // Sample the source Mesh and generate a PC.
    auto sampledPCPtr = sampleSurfaceMesh(sourceMesh_, samplingDensity_);

    // Iterate the sampledPC and compute the distance to the target Mesh.
    for (const auto& point : *sampledPCPtr) {
        // Convert to Eigen.
        const Eigen::Vector3d p(point.x, point.y, point.z);

        // Compute the distance to the Mesh. ToDo.
        double buffer[3]{p(0), p(1), p(2)};
        double minDistance = std::fabs(distance2MeshPtr->EvaluateFunction(buffer));

        // Push result
        meshDifference_.push_back(PointDistanceData(p, minDistance));
    }
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

void MeshDifference::saveHeatmapMesh(const std::string& filename,
                                     const double minDistance,
                                     const double maxDistance,
                                     const Colormap& colormap)
{
    // Check if output cloud is empty
    if (meshDifference_.empty())
        return;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr heatmapPCPtr(new pcl::PointCloud<pcl::PointXYZRGB>());

    for (const auto& heatmapEntry : meshDifference_) {
        // Extract position.
        const Eigen::Vector3d position = heatmapEntry.point;

        // Extract distance.
        const double distance = heatmapEntry.distance;

        // Scale distance in the [0, 1] range.
        const double scaledDistance =
            std::max(std::min((distance - minDistance) / (maxDistance - minDistance), 1.0), 0.0);

        // Map to color and push to the pointcloud.
        const tinycolormap::Color color = tinycolormap::GetColor(scaledDistance, colormap);

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