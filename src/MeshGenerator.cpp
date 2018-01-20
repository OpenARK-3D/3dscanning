#include "MeshGenerator.h"

using namespace std;

void MeshGenerator::run(double smoothRadius,
                        fileFormat format,
                        string inputFileName,
                        string outputFileName) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PCLPointCloud2 cloud2;
    if (format == PCD)
        pcl::io::loadPCDFile(inputFileName, cloud2);
    else
        pcl::io::loadPLYFile(inputFileName, cloud2);
    pcl::fromPCLPointCloud2(cloud2, *cloud);
    cout << "Point size: " << cloud->points.size() << endl;

    generateMesh(cloud, smoothRadius, outputFileName);
    return;
}

void MeshGenerator::run(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcloud,
                        double smoothRadius,
                        string outputFileName) {
    cout << "Point size: " << pcloud->points.size() << endl;

    generateMesh(pcloud, smoothRadius, outputFileName);
    return;
}

void MeshGenerator::generateMesh(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
                                 double smoothRadius,
                                 string outputFileName) {
    // Denoise
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(100);
    sor.setStddevMulThresh(0.05);
    sor.filter(*cloud_filtered);
    // Translate to origin
    pcl::CentroidPoint<pcl::PointXYZRGBA> centroid;
    for (int i = 0; i < cloud_filtered->points.size(); ++i)
        centroid.add(cloud_filtered->points[i]);
    pcl::PointXYZRGBA pc;
    centroid.get(pc);
    Eigen::Matrix4f transform_mat = Eigen::Matrix4f::Identity();
    transform_mat(0, 3) = -pc.x;
    transform_mat(1, 3) = -pc.y;
    transform_mat(2, 3) = -pc.z;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::transformPointCloud(*cloud_filtered, *cloud_transformed, transform_mat);

    clock_t begin, end;
    /*========================== Mesh Generation Part Starts =======================*/
    cout << "Start smoothing" << endl;
    begin = clock();
    pcl::MovingLeastSquares<pcl::PointXYZRGBA, pcl::PointXYZRGBA> mls;
    mls.setInputCloud(cloud_transformed);
    mls.setSearchRadius(smoothRadius);
    mls.setPolynomialFit(true);
    mls.setPolynomialOrder(2);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_smoothed(new pcl::PointCloud<pcl::PointXYZRGBA>());
    mls.process(*cloud_smoothed);
    end = clock();
    cout << "Smoothing finished" << endl;
    cout << "Time: " << (double)(end - begin) / CLOCKS_PER_SEC << endl;

    cout << "Start normal estimation" << endl;
    begin = clock();
    // Normal estimation*
    pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
    tree->setInputCloud(cloud_smoothed);
    n.setInputCloud(cloud_smoothed);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);
    //* normals should contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed_copy(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_smoothed, *cloud_smoothed_copy);
    pcl::concatenateFields(*cloud_smoothed_copy, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals
    end = clock();
    cout << "Normal estimation finished" << endl;
    cout << "Time: " << (double)(end - begin) / CLOCKS_PER_SEC << endl;

    cout << "Start generating mesh" << endl;
    begin = clock();
    // Initialize objects
    pcl::Poisson<pcl::PointNormal> poisson;
    pcl::PolygonMesh mesh;
    poisson.setDepth(5);
    poisson.setInputCloud(cloud_with_normals);
    poisson.reconstruct(mesh);

    end = clock();
    cout << "Mesh generated." << endl;
    cout << "Time: " << (double)(end - begin) / CLOCKS_PER_SEC << endl;

    //pcl::io::savePLYFile(outputFileName, mesh);

    pcl::PointCloud<pcl::PointXYZ> cloudData;
    pcl::fromPCLPointCloud2(mesh.cloud, cloudData);
    pcl::PointCloud<pcl::PointXYZRGBA> cloudRGB;
    for (size_t i = 0; i < cloudData.points.size(); ++i) {
        pcl::PointXYZRGBA p;
        p.x = cloudData.points[i].x;
        p.y = cloudData.points[i].y;
        p.z = cloudData.points[i].z;
        /* Edit this part to get true texture of the mesh */
        /*================================================*/
        double dis = DBL_MAX;
        int idx = 0;
        for (size_t j = 0; j < cloud_smoothed->points.size(); ++j) {
            double d = sqrt((p.x - cloud_smoothed->points[j].x) * (p.x - cloud_smoothed->points[j].x)
                            + (p.y - cloud_smoothed->points[j].y) * (p.y - cloud_smoothed->points[j].y)
                            + (p.z - cloud_smoothed->points[j].z) * (p.z - cloud_smoothed->points[j].z));
            if (dis > d) {
                dis = d;
                idx = j;
            }
        }
        p.r = cloud_smoothed->points[idx].r;
        p.g = cloud_smoothed->points[idx].g;
        p.b = cloud_smoothed->points[idx].b;
        /*================================================*/
        cloudRGB.push_back(p);
    }

    cout << "Start writing file." << endl;
    // Write to .ply file
    ofstream plyFile;
    plyFile.open(outputFileName);
    plyFile << "ply\nformat ascii 1.0\ncomment stanford bunny\nelement vertex ";
    plyFile << cloudRGB.points.size() << "\n";
    plyFile << "property float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\n";
    plyFile << "element face " << mesh.polygons.size() << "\n";
    plyFile << "property list uchar int vertex_index\nend_header\n";
    for (auto c : cloudRGB.points) {
        plyFile << c.x << " " << c.y << " " << c.z << " " << (int)c.r << " " << (int)c.g << " " << (int)c.b << "\n";
    }
    for (auto f : mesh.polygons) {
        plyFile << "3 " << f.vertices[0] << " " << f.vertices[2] << " " << f.vertices[1] << "\n";
    }
    plyFile.close();
    cout << "File saved." << endl;
    /*========================== Mesh Generation Part Ends =========================*/
    return;
}