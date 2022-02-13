
#include <string>
#include "open3d/Open3D.h"

using namespace open3d;
using namespace std;

int main(int argc, char *argv[]) {

    // load ply
    auto pcd = io::CreatePointCloudFromFile("/Users/veronika/Desktop/input_data.ply");
    visualization::DrawGeometries({pcd}, "Loaded ply");

    // down sample
    auto downpcd = pcd->VoxelDownSample(0.02);
    visualization::DrawGeometries({downpcd}, "Downsampled");

    // compute normals
    downpcd->EstimateNormals(geometry::KDTreeSearchParamHybrid(0.1, 30));
    visualization::DrawGeometries({downpcd}, "Computed normals");

    // orient normals
    auto loc = Eigen::Vector3d(0.0, 0.0, 0.0);
    downpcd->OrientNormalsTowardsCameraLocation(loc);
    visualization::DrawGeometries({downpcd}, "Oriented normals");

    // reconstruct via Poisson
    shared_ptr<geometry::PointCloud> source = downpcd;
    auto res = geometry::TriangleMesh::CreateFromPointCloudPoisson(*source);
    auto mesh = get<0>(res);
    visualization::DrawGeometries({mesh}, "Reconstructed via Poisson");

    // remove vertices with low density
    auto densities = get<1>(res);

    vector<bool> mask(densities.size());
    for (int i = 0; i < densities.size(); ++i) {
        mask[i] = densities[i] < 6.0;
    }
    mesh->geometry::TriangleMesh::RemoveVerticesByMask(mask);
    visualization::DrawGeometries({mesh}, "Removed low density vertices");

    // save
    io::WriteTriangleMesh("poisson_mesh.ply", *mesh, true, true);

    return 0;
}
