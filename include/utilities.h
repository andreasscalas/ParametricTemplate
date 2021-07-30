#ifndef UTILITIES_H
#define UTILITIES_H

#include <vector>
#include <map>
#include <Eigen/Dense>
#include <extendedtrimesh.h>
#include <vtkActor.h>
#include <vtkSmartPointer.h>

namespace Utilities {

    static const double EPSILON = 1E-5;
    static const double ZERO_EPSILON = 1E-10;
    static const double ANGLE_EPSILON = 8E-1;
    static const double STEP_SIZE = 0.1;
    static const unsigned int MAX_KMEANS_ITERATIONS = 100;
    static const unsigned int THREADS_NUMBER = 8;
    static const short BBOX_SPHERE_RATIO = 20000;
    static const unsigned int LOCAL_BOUNDARY_DIM = 7;
    static int CRITICAL_POINT = 666;
    enum type{SUPERFICIAL, VOLUMETRIC};
    enum ConstraintType {EDGE_STRAIN, TRIANGLE_STRAIN, TETRAHEDRON_STRAIN, AREA, VOLUME, BENDING, CLOSENESS, LINE, PLANE,
          CIRCLE, SPHERE, SIMILARITY, RIGID, RECTANGLE, PARALLELOGRAM, LAPLACIAN, LAPLACIAN_DISPLACEMENT};
    Eigen::Matrix4d xRotationMatrix(double alfa);
    Eigen::Matrix4d yRotationMatrix(double beta);
    Eigen::Matrix4d zRotationMatrix(double gamma);
    Eigen::Matrix4d translationMatrix(Eigen::Vector3d direction);

    std::vector<IMATI_STL::Point*> transformVertexList(std::vector<IMATI_STL::Point*> list, Eigen::Matrix4d transformationMatrix);
    /**
     * @brief mod method that computes the positive modulus
     * @param val dividend
     * @param m divisor
     * @return positive remainder of the integer division between val and m:
     * (mod(-1, 3) = 2 not mode(-1,3)=-1
     */
    int mod(int val, int m);

    /**
     * @brief isPointInsidePolygon is a method for checking if a point (v) is inside a polygon, defined as an ordered set of vertices (boundary)
     * @param v The point
     * @param boundary The polygon
     * @return true if the point is inside the polygon, false otherwise
     */
    bool isPointInsidePolygon(IMATI_STL::Point* v, std::vector<IMATI_STL::Point*> boundary);

    /**
     * @brief sign method that computes the sign of a given number
     * @param val the number of which discover the sign
     * @return the sign of the number
     */
    template <typename T> int sign(T val) { return (T(0) < val) - (val < T(0)); }

    void findFaces(std::vector<IMATI_STL::Triangle*> &faces, std::vector<IMATI_STL::Vertex*> vertices);

    /**
     * @brief findOuterFaces method that find which face (triangle) in a set is the nearest to a given vertex.
     * @param faces the set of faces.
     * @param eta the vertex which the triangle is nearest.
     * @return the found triangle.
     */
    IMATI_STL::Triangle* getCloserFace(std::vector<IMATI_STL::Triangle*> faces, IMATI_STL::Vertex* eta);

    /**
     * @brief regionGrowing method that, given a contour and a seed point, performs a region growing starting
     * from the seed in order to obtain all the triangles inside the contour.
     * @param contour the contour of the region.
     * @param seed the seed point.
     * @return the list of triangles contained in the contour.
     */
    std::vector<IMATI_STL::Triangle*> regionGrowing(std::vector<IMATI_STL::Vertex*> contour, IMATI_STL::Triangle* seed);

    /**
     * @brief regionGrowing a method for computing the list of triangles that are part of the annotation identified by a list of boundaries.
     * @param boundaries The list of boundaries
     * @return the list of triangles that are part of the annotation
     */
    std::vector<IMATI_STL::Triangle*> regionGrowing(std::vector<std::vector<IMATI_STL::Vertex*> > boundaries);

    /**
     * @brief regionGrowing a method for computing the list of triangles that are part of the annotation identified by a list of boundaries.
     * @param mesh the reference mesh
     * @param boundaries The list of boundaries
     * @return the list of triangles that are part of the annotation
     */
    std::vector<unsigned int> regionGrowing(ExtendedTrimesh* mesh, std::vector<std::vector<unsigned int> > boundaries);

    bool isPointInSegment(IMATI_STL::Point p, IMATI_STL::Point a, IMATI_STL::Point b);
    bool isPointInSegment(Eigen::Vector3d p, Eigen::Vector3d a, Eigen::Vector3d b);
    ExtendedTrimesh* generateCage(ExtendedTrimesh* shape, type type);
    std::vector<unsigned long> trianglesToIDvector(ExtendedTrimesh*, std::vector<IMATI_STL::Triangle*>);
    std::pair<IMATI_STL::Point*, IMATI_STL::Point*> compute2OrthogonalVersors(IMATI_STL::Point v);
}

#endif // UTILITIES_H
