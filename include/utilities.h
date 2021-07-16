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

    /**
     * @brief isBoundaryClockwise is a method for checking the order (clockwise or counterclockwise) of a boundary, defined as an ordered set of vertices
     * @param boundary the boundary to check
     * @return true if the boundary is clockwise, false otherwise
     */
    bool isBoundaryClockwise(std::vector<IMATI_STL::Point*> boundary);

    /**
     * @brief rotateVector is a method for applying a trasformation (rotation) to a vector, given a appropriate transform matrix.
     * @param v the vector to transform
     * @return the transformed vector
     */
    IMATI_STL::Point rotateVector(IMATI_STL::Point v, Eigen::Matrix4d);

    /**
     * @brief countBoundaryEdges method for counting the number of edges which are on the boundary of the mesh.
     * @param t the triangle on which the number of boundary edges should be counted.
     * @return the number of boundary edges.
     */
    short countBoundaryEdges(IMATI_STL::Triangle* t);

    /**
     * @brief getBoundaryEdges method for extracting the edges which are on the boundary of the mesh from a triangle
     * @param t the triangle from which extracting the boundary edges
     * @return the boundary edges
     */
    std::vector<IMATI_STL::Edge*> getBoundaryEdges(IMATI_STL::Triangle *t);

    /**
     * @brief getMAStartingTriangle is a method for obtaining the better triangle with which starting the Medial Axis Transform algorithm.
     * @param slice the mesh from which extracting the triangle
     * @return
     */
    IMATI_STL::Triangle* getMAStartingTriangle(ExtendedTrimesh* slice);

    /**
     * @brief simplifyLine method for simplifying a line, defined in "Algorithms for the reduction of the Number of Points Required to Represent a Digitized Line or its Caricature"
     * @param line the line to simplify
     * @param maxError the maximum error allowed for each approximation
     * @return the simplified line
     */
    std::vector<IMATI_STL::Point*> simplifyPolyLine(std::vector<IMATI_STL::Point*> line, double maxError);

    std::pair<double*, double> circleFitting(std::vector<double*> points);

    std::pair<Eigen::Vector3d, Eigen::Vector3d> linearRegression(std::vector<Eigen::Vector3d> points);
    std::pair<Eigen::Vector3d, Eigen::Vector3d> linearRegression1(std::vector<Eigen::Vector3d> points);


    std::pair<Eigen::Vector3d, Eigen::Vector3d>  planarRegression(Eigen::Matrix3Xd points);

    std::pair<Eigen::Vector2d, double> circleFitting(std::vector<Eigen::Vector2d> points);

    std::pair<Eigen::Vector2d, Eigen::Vector2d> planarPCA(std::vector<Eigen::Vector2d> points);
    Eigen::Matrix3d spatialPCA(std::vector<Eigen::Vector3d> points);

    std::vector<double*> imatiToDoublePointRepresentation(std::vector<IMATI_STL::Vertex*> points);

    std::vector<Eigen::Vector2d> imatiToEigen2DPointRepresentation(std::vector<IMATI_STL::Point*> points);

    std::vector<Eigen::Vector3d> imatiToEigen3DPointRepresentation(std::vector<IMATI_STL::Point*> points);

    std::vector<Eigen::Vector3d> imatiToEigen3DVertexRepresentation(std::vector<IMATI_STL::Vertex*> vertices);

    std::vector<Eigen::Vector2d> extractConvexHull(std::vector<Eigen::Vector2d> points);

    bool isLeft(Eigen::Vector2d p, Eigen::Vector2d p1, Eigen::Vector2d p2);

    bool isALessThanB(std::pair<std::pair<double, double>, unsigned int> a, std::pair<std::pair<double, double>, unsigned int> b);

    double computeAngleBetweenVectors(Eigen::Vector2d v1, Eigen::Vector2d v2);
    Eigen::Matrix3d create2DTranslationMatrix(Eigen::Vector2d direction);
    Eigen::Matrix2d create2DRotationMatrix(double angle);
    Eigen::Vector2d computeLineIntersection(Eigen::Vector2d v1, Eigen::Vector2d p1, Eigen::Vector2d v2, Eigen::Vector2d p2);
    std::vector<Eigen::Vector2d> get2DOBB(std::vector<Eigen::Vector2d> points);
    std::vector<double> getOBB(ExtendedTrimesh* mesh);
    bool isGreater(double a, double b);
    bool areEqual(double a, double b);
    std::vector<std::vector<unsigned int> > kMeansCluster(std::vector<Eigen::VectorXd> points, unsigned int k);
    void renderSpline(std::vector<IMATI_STL::Point*> spline, vtkSmartPointer<vtkActor> actor);

    void removeCreeks(std::vector<IMATI_STL::Point*> &boundary, double ray);

    std::vector<IMATI_STL::Point*> getLevelSet(std::map<IMATI_STL::Point *, double> morseFunction, double);
    bool isPointInSegment(IMATI_STL::Point p, IMATI_STL::Point a, IMATI_STL::Point b);
    bool isPointInSegment(Eigen::Vector3d p, Eigen::Vector3d a, Eigen::Vector3d b);
    IMATI_STL::Point* edgePlaneIntersection(IMATI_STL::Edge e, IMATI_STL::Point normal, IMATI_STL::Point origin);

    ExtendedTrimesh* segmentMeshHead(ExtendedTrimesh* mesh);
    bool areBoundariesEqual(std::vector<IMATI_STL::Point*> a, std::vector<IMATI_STL::Point*> b);

    std::pair<Eigen::Vector2d, double> circleFrom3Points(Eigen::Vector2d p1, Eigen::Vector2d p2, Eigen::Vector2d p3);
    std::vector<Eigen::Vector2d> lineCircleIntersection(Eigen::Vector2d p1, Eigen::Vector2d p2, Eigen::Vector2d center, double ray);
    double computeLocalCurvature2D(IMATI_STL::Vertex* v, double circleRay, unsigned short neighbourhoodSize);
    std::map<IMATI_STL::Vertex*, double> computePolygonCurvature(ExtendedTrimesh* polygon);

    ExtendedTrimesh* segmentSelection(ExtendedTrimesh* mesh, std::vector<std::vector<IMATI_STL::Vertex*> > selection);

    ExtendedTrimesh* generateCage(ExtendedTrimesh* shape, type type);
    void simplifyMeshSemantically(ExtendedTrimesh* shape);

    std::vector<unsigned long> verticesToIDvector(ExtendedTrimesh*, std::vector<IMATI_STL::Vertex*>);
    std::vector<int> verticesToIntIDvector(ExtendedTrimesh*, std::vector<IMATI_STL::Vertex*>);
    std::vector<unsigned long> trianglesToIDvector(ExtendedTrimesh*, std::vector<IMATI_STL::Triangle*>);

    void polyLineFitting(std::vector<IMATI_STL::Vertex*> p1, std::vector<IMATI_STL::Vertex*> p2, int s1, int e1, int s2, int e2, std::map<unsigned int, unsigned int> &correspondences);

    std::pair<unsigned int, std::vector<IMATI_STL::Point*>> sphere_line_intersection (IMATI_STL::Point* p1, IMATI_STL::Point* p2, IMATI_STL::Point* p3, double r, bool inSegment);

    std::vector<std::pair<IMATI_STL::Point*, std::vector<IMATI_STL::Vertex*> > > extractSkeletonWithLoops(ExtendedTrimesh* mesh, std::vector<unsigned int> triangles, std::vector<std::vector<unsigned int> > boundaries, unsigned int seedLoop);
    std::vector<unsigned int> pointsToTriangles(std::vector<unsigned int> points, ExtendedTrimesh* mesh);
    std::vector<unsigned int> pointsToTriangles(std::vector<int> points, ExtendedTrimesh* mesh);
    std::vector<int> trianglesToPoints(std::vector<unsigned int> triangles, ExtendedTrimesh *mesh);
    std::vector<std::vector<IMATI_STL::Vertex *> > computeOutlines(std::vector<IMATI_STL::Triangle *> set);
    double computeMeshVolume(ExtendedTrimesh* mesh);
    std::pair<IMATI_STL::Point*, IMATI_STL::Point*> compute2OrthogonalVersors(IMATI_STL::Point v);
}

#endif // UTILITIES_H
