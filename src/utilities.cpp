#include "utilities.h"

#include <queue>
#include <stack>
#include <mutex>
#include <math.h>

#include <surfaceannotation.h>
#include <lineannotation.h>
#include <pointannotation.h>
#include <circularlist.h>

#include <MathGeoLib.h>

#include <vtkSmartPointer.h>
#include <vtkPlane.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkLine.h>
#include <vtkPolyDataMapper.h>

using namespace IMATI_STL;
using namespace std;
using namespace Eigen;
using namespace AndreasStructures;


Matrix4d Utilities::xRotationMatrix(double alfa){

    Eigen::Matrix4d xRot;

    xRot<<  1,          0,          0,          0,
            0,          cos(alfa),  sin(alfa),  0,
            0,          -sin(alfa), cos(alfa),  0,
            0,          0,          0,          1;

    return xRot;

}

Matrix4d Utilities::yRotationMatrix(double alfa){

    Eigen::Matrix4d yRot;

    yRot<<  cos(alfa),  0,          -sin(alfa), 0,
            0,          1,          0,          0,
            sin(alfa),  0,          cos(alfa),  0,
            0,          0,          0,          1;

    return yRot;

}

Matrix4d Utilities::zRotationMatrix(double alfa){

    Eigen::Matrix4d zRot;

    zRot<<  cos(alfa),  sin(alfa),  0,          0,
            -sin(alfa), cos(alfa),  0,          0,
            0,          0,          1,          0,
            0,          0,          0,          1;

    return zRot;

}

Matrix4d Utilities::translationMatrix(Vector3d direction){

    Eigen::Matrix4d translation;

    translation<<   1,       0,      0,      direction(0),
                    0,       1,      0,      direction(1),
                    0,       0,      1,      direction(2),
                    0,       0,      0,      1;

    return translation;
}

int Utilities::mod(int val, int m){ return (val % m + m) % m; }

bool Utilities::isPointInsidePolygon(Point* v, std::vector<Point*> boundary){

    int cn = 0;    // the  crossing number counter

    // loop through all edges of the polygon
    for (unsigned int i = 0; i < static_cast<unsigned int>(boundary.size() - 1); i++) {    // edge from boundary[i]  to boundary[i+1]
       if (((boundary[i]->z <= v->z) && (boundary[i+1]->z > v->z))     // an upward crossing
        || ((boundary[i]->z > v->z) && (boundary[i+1]->z <=  v->z))) { // a downward crossing
            // compute  the actual edge-ray intersect x-coordinate
            double vt = static_cast<double>((v->z - boundary[i]->z) / (boundary[i+1]->z - boundary[i]->z));
            if (v->x <  boundary[i]->x + vt * (boundary[i+1]->x - boundary[i]->x)) // v.x < intersect
                 ++cn;   // a valid crossing of y=v.y right of v.x
        }
    }
    return (cn&1);    // 0 if even (out), and 1 if  odd (in)
}

IMATI_STL::Triangle* Utilities::getCloserFace(vector<IMATI_STL::Triangle*> faces, Vertex* v){

    //This closeness concept is based on the Euclidean distance
    pair<double, IMATI_STL::Triangle*> closerFace(DBL_MAX, nullptr);

    //For each face we compute the distance from the vertex and we store only the closest one
    for(vector<IMATI_STL::Triangle*>::iterator it = faces.begin(); it != faces.end(); it++){

        IMATI_STL::Triangle* t = *it;
        double distance = t->distanceFromPoint(v);
        if(distance < closerFace.first){
            closerFace.first = distance;
            closerFace.second = t;
        }
    }

    return closerFace.second;

}

vector<IMATI_STL::Triangle*> Utilities::regionGrowing(vector<Vertex*> contour, IMATI_STL::Triangle* seed){

    queue<IMATI_STL::Triangle*> neighbors;
    vector<IMATI_STL::Triangle*> internalTriangles;
    neighbors.push(seed);
    unsigned int BOUNDARY_EDGE = 1;
    unsigned int ALREADY_USED = 9;

    for(unsigned int i = 1; i <= static_cast<unsigned int>(contour.size()); i++){

        Vertex* v1, * v2;
        if(i < contour.size()){
            v1 = contour[i - 1];
            v2 = contour[i];
        }else{
            v1 = contour[i - 1];
            v2 = contour[0];
        }

        Edge* e = v1->getEdge(v2);
        if(e != nullptr)
            e->info = &BOUNDARY_EDGE;
    }


    while(neighbors.size() > 0){
        IMATI_STL::Triangle* t = neighbors.front();
        neighbors.pop();
        Edge* e = t->e1;
        for(int i = 0; i < 3; i++){
            if(!(e->info != nullptr && *static_cast<unsigned int*>(e->info) == BOUNDARY_EDGE)){
                IMATI_STL::Triangle* t_ = e->oppositeTriangle(t);
                if(t_->info == nullptr ||  *static_cast<unsigned int*>(t_->info) != ALREADY_USED){
                    internalTriangles.push_back(t_);
                    t_->info = &ALREADY_USED;
                    neighbors.push(t_);
                }
            }
            e = t->nextEdge(e);
        }
    }

    for(unsigned int i = 1; i <= static_cast<unsigned int>(contour.size()); i++){

        Vertex* v1, *v2;
        if(i < contour.size()){
            v1 = contour[i - 1];
            v2 = contour[i];
        }else{
            v1 = contour[i - 1];
            v2 = contour[0];
        }

        Edge* e = v1->getEdge(v2);
        if(e != nullptr)
            e->info = nullptr;
    }

    for(std::vector<IMATI_STL::Triangle*>::iterator it = internalTriangles.begin(); it != internalTriangles.end(); it++)
        (*it)->info = nullptr;


    return internalTriangles;

}

/**
 * @brief regionGrowing a method for computing the list of triangles that are part of the annotation identified by a list of boundaries.
 * @param boundaries The list of boundaries
 * @return the list of triangles that are part of the annotation
 */
vector<IMATI_STL::Triangle*> regionGrowing(vector<vector<Vertex*> > boundaries){

    /* neighbors is a queue that stores the list of available triangles. A queue is a data structure
     * that follows the FIFO service order, see https://en.wikipedia.org/wiki/FIFO_(computing_and_electronics).
     * So, when selecting which triangle to use from the list of available triangles, you
     * always take the one that has been inside the queue for more time.
     */
    queue<IMATI_STL::Triangle*> neighbors;
    //List of triangles of the annotation (initially empty)
    vector<IMATI_STL::Triangle*> internalTriangles;
    /* Value that says when an edge is on the boundary (if the value in its "info" field is equal to 1,
     * then the edge is on the boundary). I don't know if you can add attributes to edges in Unity, so maybe it
     * is better if you just use the method with 3 lists that we used in the example
     */
    unsigned int BOUNDARY_EDGE = 1;
    /* Value that says when a triangle has already been used (if the value in its "info" field is equal to 1,
     * then the edge is on the boundary). Again, I don't know if you can add attributes to triangle in Unity, so
     * maybe it better if you just use the method with 3 lists that we used in the example
     */
    unsigned int ALREADY_USED = 9;

    //We iterate over the list of boundaries
    for(unsigned int i = 0; i < boundaries.size(); i++){

        //This is the i-th boundary
        vector<Vertex*> boundary = boundaries[i];
        /* For each boundary, we take the first two vertices (boundary[0] and boundary[1]) in the boundary.
         * The boundary is ordered counterclockwise, so you have to consider the triangle (t) on the left of
         * the edge connecting the two vertices.
         */
        IMATI_STL::Triangle* t = boundary[0]->getEdge(boundary[1])->leftTriangle(boundary[0]);
        internalTriangles.push_back(t);
        t->info = static_cast<void*>(&ALREADY_USED);
        //We insert the triangle (or seed of the region growing algorithm) in the queue of available triangles
        neighbors.push(t);

        //We mark each edge in the boundary as a boundary edge
        for(unsigned int i = 1; i <= boundary.size() - 1; i++){

            Vertex* v1 = boundary[i - 1];
            Vertex* v2 = boundary[i]; //% is the modulo operator. If i is equal to boundary.size() we use boundary[0].

            Edge* e = v1->getEdge(v2);
            if(e != nullptr)
                e->info = static_cast<void*>(&BOUNDARY_EDGE);
            else {
                std::cerr << "This shouldn't happen. Error in the identification of the edge." << std::endl << std::flush;
                exit(5);
            }
        }
    }

    //We will analyse triangles until the queue of available triangles is not empty.
    while(neighbors.size() > 0){
        //We take the first triangle (t) from the queue and remove it from there
        IMATI_STL::Triangle* t = neighbors.front();
        neighbors.pop();
        //We take the first edge (e) of t
        Edge* e = t->e1;
        //t has 3 edges, so we will perform the analysis 3 times
        for(int i = 0; i < 3; i++){
            //If e is not on the boundary
            if(e->info == nullptr || *static_cast<unsigned int*>(e->info) != BOUNDARY_EDGE){
                //We take the triangle (t_) on the opposite side of e with respect to t.
                IMATI_STL::Triangle* t_ = e->oppositeTriangle(t);
                if(t_ == nullptr)
                    continue;   //the mesh is not watertight and the edge is on the bounday of the mesh
                //If t_ has't been used yet, we insert it in the queue of usable triangles and in the list of triangles that are part of the annotation
                if(t_->info == nullptr ||  *static_cast<unsigned int*>(t_->info) != ALREADY_USED){
                    internalTriangles.push_back(t_);
                    t_->info = static_cast<void*>(&ALREADY_USED);
                    neighbors.push(t_);
                }
            }
            //We take the next edge of t.
            e = t->nextEdge(e);
        }
    }

    //The actual algorithm is ended.

    //Ignore this for cycle, I am just removing the markings from the edges
    for(unsigned int i = 0; i < boundaries.size(); i++){

        vector<Vertex*> boundary = boundaries[i];
        for(unsigned int i = 1; i <= boundary.size() - 1; i++){

            Vertex* v1 = boundary[i - 1];
            Vertex* v2 = boundary[i];

            Edge* e = v1->getEdge(v2);
            if(e != nullptr)
                e->info = nullptr;
        }
    }


    //Ignore this for cycle, I am just removing the markings from the triangles
    for(std::vector<IMATI_STL::Triangle*>::iterator it = internalTriangles.begin(); it != internalTriangles.end(); it++)
        (*it)->info = nullptr;


    //Now the list has all the triangles in the annotation.
    return internalTriangles;

}

IMATI_STL::Point Utilities::rotateVector(IMATI_STL::Point v, Matrix4d rotationMatrix){

    Vector4d n = {v.x, v.y, v.z, 1};
    Vector4d n_ = rotationMatrix * n;
    Point newNormal(n_(0), n_(1), n_(2));
    return newNormal;

}

bool Utilities::isBoundaryClockwise(std::vector<Point*> boundary){

    double sum = 0.0;
    for (unsigned int i = 0; i < static_cast<unsigned int>(boundary.size()); i++) {
        Point v1 = boundary[i];
        Point v2 = boundary[(i + 1) % boundary.size()]; // % is the modulo operator
        sum += (v2.x - v1.x) * (v2.z + v1.z);
    }

    return sum > 0.0;

}

vector<Edge*> Utilities::getBoundaryEdges(IMATI_STL::Triangle* t){
    vector<Edge*> boundaryEdges;
    if(t->e1->isOnBoundary())
        boundaryEdges.push_back(t->e1);
    if(t->e2->isOnBoundary())
        boundaryEdges.push_back(t->e2);
    if(t->e3->isOnBoundary())
        boundaryEdges.push_back(t->e3);

    return boundaryEdges;
}

IMATI_STL::Triangle* Utilities::getMAStartingTriangle(ExtendedTrimesh *slice){
    IMATI_STL::Triangle* t = nullptr, *starting = nullptr, *lastBranching = nullptr, *lastSimple = nullptr;
    IMATI_STL::Node* n;
    bool found = false;

    for(n = slice->T.head(); n != nullptr; n = n->next()){

        t = static_cast<IMATI_STL::Triangle*>(n->data);
        short numBoundaryEdges = static_cast<short>(getBoundaryEdges(t).size());

        switch(numBoundaryEdges){
            case 3:
            case 2:
                found = true;
                starting = t;
                break;
            case 1:
                lastSimple = t;
                break;
            case 0:
                lastBranching = t;
            break;
            default:
                ;
        }

        if(found)
            break;
    }

    if(starting == nullptr){
        if(lastBranching == nullptr)
            starting = lastSimple;
        else
            starting = lastBranching;
    }

    return starting;
}

std::vector<Point *> Utilities::simplifyPolyLine(std::vector<Point *> line, double maxError){

    unsigned int anchorPosition = 0;
    unsigned int floaterPosition = static_cast<unsigned int>(Utilities::mod(static_cast<int>(anchorPosition - 1), static_cast<int>(line.size())));
    unsigned int  startingFloater = floaterPosition;
    Point* anchor = line[anchorPosition];
    Point* floater = line[floaterPosition];
    vector<Point*> newContour = {anchor};
    stack<unsigned int> floaters;
    floaters.push(floaterPosition);

    while(anchorPosition != startingFloater){

        double worstDistance = 0;
        int worstPosition = -1;

        for(unsigned int i = (anchorPosition + 1) % (line.size()); i != floaterPosition; i = (i + 1) % (line.size())){
            Point* v = line[i];
            if(anchor != floater){
                double distance = v->distanceFromLine(anchor, floater);
                if(distance > maxError && distance > worstDistance){
                    worstPosition = static_cast<int>(i);
                    worstDistance = distance;
                }
            }
        }

        if(worstPosition != -1){

            floaterPosition = static_cast<unsigned int>(worstPosition);
            floater = line[floaterPosition];
            floaters.push(floaterPosition);

        }else{

            anchorPosition = floaterPosition;
            anchor = line[anchorPosition];
            newContour.push_back(anchor);
            floaterPosition = floaters.top();
            if(floaters.top() == anchorPosition)
                floaters.pop();

            if(floaters.size() > 0)
                floaterPosition = floaters.top();
            else
                break;

            floater = line[floaterPosition];

        }


    }

    line.push_back(line.front());
    newContour.push_back(newContour.front());
    return newContour;

}

std::pair<double*, double> Utilities::circleFitting(std::vector<double*> points){

    double xc = 7;
    double yc = 5;
    double r = 0;
    double fold = DBL_MAX;
    double fnew = DBL_MAX;

    double alpha = 0.5;
    do{
        fold = fnew;
        double xcgradient = 0, ycgradient = 0, rgradient = 0;
        for(unsigned int i = 1; i < points.size(); i++){
            double distanceFromCenter = sqrt(pow(points[i][0] - xc, 2) + pow(points[i][1] - yc, 2));
            rgradient += r - distanceFromCenter;
            xcgradient += (points[i][0] - xc) * (r - distanceFromCenter) / distanceFromCenter;
            ycgradient += (points[i][1] - yc) * (r - distanceFromCenter) / distanceFromCenter;
        }

        r -= alpha * rgradient / points.size();
        xc -= alpha * xcgradient / points.size();
        yc -= alpha * ycgradient / points.size();
        fnew = 0;
        for(unsigned int i = 0; i < points.size(); i++){
            fnew += pow(r - sqrt(pow(points[i][0] - xc, 2) + pow(points[i][1] - yc, 2)), 2);
        }

        fnew /= 2 * points.size();

    }while(fold > fnew + EPSILON);

    double* center = static_cast<double*>(malloc(2 * sizeof(double)));
    center[0] = xc;
    center[1] = yc;
    return std::make_pair(center, r);

}

std::pair<Eigen::Vector2d, double> Utilities::circleFitting(std::vector<Vector2d> points){

    Vector2d center = {0, 0};
    double radius = 0;
    double fold = DBL_MAX;
    double fnew = DBL_MAX;

    for(unsigned int i = 1; i < points.size(); i++)
        center += points[i];
    center /= points.size();
    radius = (points[0] - center).norm();

    double alpha = STEP_SIZE;
    do{
        fold = fnew;
        Vector2d cgradient = {0, 0};
        double rgradient = 0;
        for(unsigned int i = 1; i < points.size(); i++){

            double distanceFromCenter = (points[i] - center).norm();
            rgradient += radius - distanceFromCenter;
            cgradient += (points[i] - center) * (radius - distanceFromCenter) / distanceFromCenter;
        }

        radius -= alpha * rgradient / points.size();
        center -= alpha * cgradient / points.size();
        fnew = 0;

        for(unsigned int i = 0; i < points.size(); i++){
            fnew += pow(radius - (points[i] - center).norm(), 2);
        }

        fnew /= 2 * points.size();

    }while(fold > fnew + EPSILON);

    return std::make_pair(center, radius);

}

std::pair<Vector3d, Vector3d> Utilities::linearRegression(std::vector<Vector3d> points){

    unsigned int m = static_cast<unsigned int>(points.size());
    Vector3d p0 = points[0];
    Vector3d u = {0, 1, 0};
    double t = 1;

    double alpha = 0.005;
    double fold = DBL_MAX;
    double fnew = DBL_MAX;
    do{

        Vector3d p0Gradient = {0, 0, 0};
        Vector3d uGradient = {0, 0, 0};
        double tGradient = 0;
        fold = fnew;

        for(unsigned int i = 0; i < m; i++){

            Vector3d g = points[i] - p0 - u * t;
            p0Gradient += g;
            uGradient += g * t;
            tGradient += g.dot(u);

        }

        p0Gradient /= -m;
        uGradient /= -m;
        tGradient /= -m;
        p0 -= alpha * p0Gradient;
        u -= alpha * uGradient;
        t -= alpha * tGradient;

        fnew = 0;

        for(unsigned int i = 0; i < points.size(); i++)
            fnew += pow((points[i] - p0 - u * t).norm(), 2);

        fnew /= 2 * m;
        u /= u.norm();

    }while(fold > fnew + 10E-20);

    return std::make_pair(u, p0);

}

std::pair<Vector3d, Vector3d> Utilities::linearRegression1(std::vector<Vector3d> points){

    unsigned int m = static_cast<unsigned int>(points.size());
    Vector3d p0 = points[0];
    p0(1) = 0;
    Vector3d u = {0, 1, 0};

    double alpha = 0.005;
    double fold = DBL_MAX;
    double fnew = DBL_MAX;
    do{

        Vector3d p0Gradient = {0, 0, 0};
        Vector3d uGradient = {0, 0, 0};
        fold = fnew;

        for(unsigned int i = 0; i < m; i++){

            double t = (points[i](0) - p0(0)) * u(0) + (points[i](1) - p0(1)) * u(1) + (points[i](2) - p0(2)) * u(2);
            Vector3d g = points[i] - p0 - u * t;
            p0Gradient += g;
            uGradient += g * t;

        }

        p0Gradient /= -m;
        uGradient /= -m;
        p0 -= alpha * p0Gradient;
        u -= alpha * uGradient;

        fnew = 0;

        for(unsigned int i = 0; i < points.size(); i++){
            double t = (points[i](0) - p0(0)) * u(0) + (points[i](1) - p0(1)) * u(1) + (points[i](2) - p0(2)) * u(2);
            fnew += pow((points[i] - p0 - u * t).norm(), 2);
        }

        fnew /= 2 * m;
        u /= u(1);
        p0(1) = 0;

    }while(fold > fnew + 10E-20);

    return std::make_pair(u, p0);

}

std::vector<double *> Utilities::imatiToDoublePointRepresentation(vector<Vertex*> points){

    vector<double*> newVector;

    for(unsigned int i = 0; i < points.size(); i++){
        double* p = static_cast<double*>(malloc(2 * sizeof(double)));
        p[0] = points[i]->x;
        p[1] = points[i]->z;
        newVector.push_back(p);
    }

    return newVector;
}

std::vector<Vector2d> Utilities::imatiToEigen2DPointRepresentation(std::vector<Point*> points){

    vector<Vector2d> newVector;

    for(unsigned int i = 0; i < static_cast<unsigned int>(points.size()); i++){
        Vector2d p = {points[i]->x, points[i]->z};
        newVector.push_back(p);
    }

    return newVector;
}

std::vector<Vector3d> Utilities::imatiToEigen3DPointRepresentation(std::vector<Point*> points)
{
    vector<Vector3d> newVector;

    for(unsigned int i = 0; i < points.size(); i++){
        Vector3d p = {points[i]->x, points[i]->y, points[i]->z};
        newVector.push_back(p);
    }

    return newVector;
}


std::vector<Vector3d> Utilities::imatiToEigen3DVertexRepresentation(std::vector<Vertex*> vertices)
{
    vector<Vector3d> newVector;

    for(unsigned int i = 0; i < vertices.size(); i++){
        Vector3d p = {vertices[i]->x, vertices[i]->y, vertices[i]->z};
        newVector.push_back(p);
    }

    return newVector;
}

std::pair<Vector2d, Vector2d> Utilities::planarPCA(std::vector<Vector2d> points){

    Vector2d mean;
    for(unsigned int i = 0; i < points.size(); i++)
        mean += points[i];

    mean /= points.size();

    Matrix2d sigma;
    sigma(0,0) = 0;
    sigma(0,1) = 0;
    sigma(1,0) = 0;
    sigma(1,1) = 0;

    for(unsigned int i = 0; i < points.size(); i++){
        sigma += (points[i] - mean) * ((points[i] - mean).transpose());
    }

    sigma /= points.size();
    JacobiSVD<MatrixXd> svd(sigma, ComputeFullU | ComputeFullV);
    MatrixXd U = svd.matrixU();
    Vector2d p1 = {U(0,0), U(0,1)};
    Vector2d p2 = {U(1,0), U(1,1)};


    return std::make_pair(p1, p2);

}

Eigen::Matrix3d Utilities::spatialPCA(std::vector<Eigen::Vector3d> points){

    Vector3d mean;
    for(unsigned int i = 0; i < points.size(); i++)
        mean += points[i];

    mean /= points.size();

    Matrix3d sigma;
    sigma << 0, 0, 0,
             0, 0, 0,
             0, 0, 0;

    for(unsigned int i = 0; i < points.size(); i++){
        sigma += (points[i] - mean) * ((points[i] - mean).transpose());
    }

    sigma /= points.size();
    JacobiSVD<MatrixXd> svd(sigma, ComputeFullU | ComputeFullV);
    Matrix3d U = svd.matrixU();
    return U;
}

std::vector<double> Utilities::getOBB(ExtendedTrimesh* mesh){

    vector<vec> pointsVector;
    vector<double> bBPoints;
    IMATI_STL::Node* n;
    Vertex* v;
    for(n = mesh->V.head(); n != nullptr; n=  n->next()){
        v = static_cast<Vertex*>(n->data);
        vec p = POINT_VEC(static_cast<float>(v->x), static_cast<float>(v->y), static_cast<float>(v->z));
        pointsVector.push_back(p);
    }

    vec* pv = pointsVector.data();
    OBB minOBB = OBB::OptimalEnclosingOBB(pv, static_cast<int>(pointsVector.size()));

    for(unsigned int i = 0; i < 8; i++){
        bBPoints.push_back(static_cast<double>(minOBB.CornerPoint(static_cast<int>(i)).x));
        bBPoints.push_back(static_cast<double>(minOBB.CornerPoint(static_cast<int>(i)).y));
        bBPoints.push_back(static_cast<double>(minOBB.CornerPoint(static_cast<int>(i)).z));
    }

    return bBPoints;
}

bool Utilities::isALessThanB(std::pair<std::pair<double, double>, unsigned int> a, std::pair<std::pair<double, double>, unsigned int> b){

    if(a.first.first < b.first.first)
        return true;
    else if(Utilities::areEqual(a.first.first,b.first.first) && a.first.second < b.first.second)
        return true;
    return false;

}

bool Utilities::isLeft(Vector2d p, Vector2d p1, Vector2d p2){
    return (p(0) - p1(0)) * (p2(1) - p1(1)) - (p(1) - p1(1)) * (p2(0) - p1(0)) < 0;
}

std::vector<Vector2d> Utilities::extractConvexHull(std::vector<Vector2d> points){

    //This function is based on the Graham's Algorithm
    unsigned int lowestLeftmostPosition = 0;
    double smallestY = DBL_MAX, smallestX = DBL_MAX;

    vector<pair<pair<double, double>, unsigned int> > segmentsAngleAndLength;
    for(unsigned int i = 0; i < static_cast<unsigned int>(points.size()); i++)
        if(points[i](1) < smallestY){
            lowestLeftmostPosition = i;
            smallestY = points[i](1);
        }else if(areEqual(points[i](1), smallestY) && points[i](0) < smallestX){
            lowestLeftmostPosition = i;
            smallestX = points[i](0);
        }

    Vector2d xAxis = {1, 0};
    for(unsigned int i = 0; i < static_cast<unsigned int>(points.size()); i++){

        if(i == lowestLeftmostPosition)
            continue;

        Vector2d pi = points[i];
        Vector2d v = pi - points[lowestLeftmostPosition];
        v /= v.norm();
        double angle = computeAngleBetweenVectors(xAxis, v);

        pair<double, double> angleAndLength = make_pair(angle, v.norm());
        pair<pair<double, double>, unsigned int> segmentAngleAndLength = make_pair(angleAndLength, i);
        segmentsAngleAndLength.push_back(segmentAngleAndLength);
    }

    sort(segmentsAngleAndLength.begin(), segmentsAngleAndLength.end(), isALessThanB);
    vector<Vector2d> convexHull;
    convexHull.push_back(points[lowestLeftmostPosition]);
    convexHull.push_back(points[segmentsAngleAndLength[0].second]);

    unsigned int i = 1;
    while(i < segmentsAngleAndLength.size()){

        unsigned int actualPointPosition = segmentsAngleAndLength[i].second;
        Vector2d p = points[actualPointPosition];
        Vector2d p1 = convexHull.at(convexHull.size() - 2);
        Vector2d p2 = convexHull.back();
        if(isLeft(p, p1, p2)){
            convexHull.push_back(p);
            i++;
        }else
            convexHull.pop_back();


    }

    convexHull.push_back(convexHull.front());

    return convexHull;

}

std::vector<Vector2d> Utilities::get2DOBB(std::vector<Vector2d> points){

    vector<Eigen::Vector2d> ch = Utilities::extractConvexHull(points);

    //This function is based on the Rotating Calipers Algorithm
    double minX = DBL_MAX, maxX = -DBL_MAX, minY = DBL_MAX, maxY = -DBL_MAX;
    vector<Vector2d> pivots = {{0,0}, {0,0}, {0,0}, {0,0}};
    vector<unsigned int> pivotPositions = {UINT16_MAX, UINT16_MAX, UINT16_MAX};

    for(unsigned int i = 0; i < static_cast<unsigned int>(ch.size()); i++){
        if(ch[i](1) < minY){
            minY = ch[i](1);
            pivotPositions[0] = i;
            pivots[0] = ch[i];
        }
        if(ch[i](0) > maxX){
            maxX = ch[i](0);
            pivotPositions[1] = i;
            pivots[1] = ch[i];
        }
        if(ch[i](1) > maxY){
            maxY = ch[i](1);
            pivotPositions[2] = i;
            pivots[2] = ch[i];
        }
        if(ch[i](0) < minX){
            minX = ch[i](0);
            pivotPositions[3] = i;
            pivots[3] = ch[i];
        }
    }

    vector<Vector2d> bbs = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};

    double area = (maxX - minX) * (maxY - minY);

    vector<Vector2d> rv{{maxX, minY}, {maxX, maxY}, {minX, maxY}, {minX, minY}};
    double performedRotation = 0;

    for(unsigned int j = 0; j < static_cast<unsigned int>(ch.size() - 1); j++){

        vector<unsigned int> nextPositions;
        vector<Vector2d> vs;
        vector<Vector2d> intersections;
        double minAngle = DBL_MAX;
        unsigned int minAnglePivot;

        for(unsigned int i = 0; i < 4; i++){
            pivots[i] = ch[pivotPositions[i]];
            nextPositions.push_back((pivotPositions[i] + 1) % ch.size());
            vs.push_back(ch[nextPositions[i]] - pivots[i]);
            vs[i] /= vs[i].norm();
            double angle = computeAngleBetweenVectors(bbs[i], vs[i]);
            if(angle < minAngle){
                minAngle = angle;
                minAnglePivot = i;
            }
        }

        Matrix2d rotationMatrix = create2DRotationMatrix(minAngle);

        for(unsigned int i = 0; i < 4; i++)
            bbs[i] = rotationMatrix * bbs[i];

        for(unsigned int i = 0; i < 4; i++)
            intersections.push_back(computeLineIntersection(bbs[i], pivots[i], bbs[(i + 1) % 4], pivots[(i + 1) % 4]));

        double newArea = (intersections[1] - intersections[0]).norm() *
                         (intersections[2] - intersections[1]).norm();

        if(newArea < area){
            area = newArea;
            for(unsigned int i = 0; i < 4; i++)
                rv[i] = intersections[i];
        }

        pivotPositions[minAnglePivot] = nextPositions[minAnglePivot];
        performedRotation += minAngle;


    }


    return rv;
}

double Utilities::computeAngleBetweenVectors(Vector2d v1, Vector2d v2){

    double angle = acos(v1.dot(v2) / (v1.norm() * v2.norm()));
    return angle;

}

Matrix2d Utilities::create2DRotationMatrix(double angle){

    Matrix2d rotationMatrix;
    rotationMatrix << cos(angle), -sin(angle),
                      sin(angle), cos(angle);

    return rotationMatrix;

}

Vector2d Utilities::computeLineIntersection(Vector2d v1, Vector2d p1, Vector2d v2, Vector2d p2){

    double x;
    double a, b, c, d;
    bool aUndefined = false, bUndefined = false;

    if(areEqual(v1(0), 0)){

        x = p1(0);
        aUndefined = true;

    }else{

        a = v1(1) / v1(0);
        c = (-a) * p1(0) + p1(1);

    }

    if(areEqual(v2(0), 0)){

        x = p2(0);
        bUndefined = true;

    }else{

        b = v2(1) / v2(0);
        d = (-b) * p2(0) + p2(1);

    }

    if(!(aUndefined || bUndefined))
        x = (d - c) / (a - b);

    double y;

    if(aUndefined)
        y = b * x + d;
    else
        y = a * x + c;

    Vector2d intersection = {x, y};
    return intersection;
}

std::vector<Point*> Utilities::transformVertexList(std::vector<IMATI_STL::Point *> list, Matrix4d transformationMatrix){

    vector<Point*> transformedPoints;
    for(unsigned int i = 0; i < list.size(); i++){
        Vector4d p = {list[i]->x, list[i]->y, list[i]->z, 1};
        Vector4d tr = transformationMatrix * p;
        Point* tp = new Point(tr(0), tr(1), tr(2));
        transformedPoints.push_back(tp);
    }

    return transformedPoints;

}

bool Utilities::isGreater(double a, double b){
    return a > b ? true : false;
}


void Utilities::renderSpline(std::vector<Point*> spline, vtkSmartPointer<vtkActor> actor){
    vtkSmartPointer<vtkPolyData> data = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

    points->InsertNextPoint(spline[0]->x, spline[0]->y, spline[0]->z);

    for(unsigned int i = 1; i < spline.size(); i++){

        points->InsertNextPoint(spline[i]->x, spline[i]->y, spline[i]->z);
        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetId(0, i - 1);
        line->GetPointIds()->SetId(1, i);
        lines->InsertNextCell(line);

    }

    data->SetPoints(points);
    data->SetLines(lines);

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(data);
    actor->SetMapper(mapper);
}

bool Utilities::areEqual(double a, double b){
    bool result = false;
    if((a + ZERO_EPSILON > b) && (a - ZERO_EPSILON < b))
        result = true;

    return result;
}

void Utilities::removeCreeks(std::vector<Point*> &boundary, double ray)
{

    CircularList<Point*> contour;
    std::vector<Point*> newBoundary;
    for(unsigned int i = 0; i < boundary.size() - 1; i++)
        contour.insertAfter(boundary[i]);

    for(unsigned int i = 0; i < boundary.size() - 1; i++){
        Point* p = boundary[i];
        int p_index = mod(i - 1, boundary.size() - 1);
        int n_index = mod(i + 1, boundary.size() -1);
        Point* prev = boundary[p_index];
        Point* next = boundary[n_index];
        Point* n;
        Vector2d e_prev = {prev->x, prev->z};
        Vector2d e_p = {p->x, p->z};
        Vector2d e_next = {next->x, next->z};
        Point v1 = (*prev - *p);
        Point v2 = (*next - *p);
        v1.normalize();
        v2.normalize();

        n = new Point(v1 + v2);
        Point* tmp = new Point((*next + (*prev)) / 2);
        if(isPointInsidePolygon(tmp, boundary))
            *n *= -1;

        n->normalize();

        Point* sphereCenter = new Point(*p + (*(n) * ray));

        /*cout<<"P_p: ("<<prev->x<<", "<< prev->z<<")"<<endl<<flush;
        cout<<"P: ("<<p->x<<", "<< p->z<<")"<<endl<<flush;
        cout<<"P_n: ("<<next->x<<", "<< next->z<<")"<<endl<<flush;
        cout<<"n: ("<<n->x<<", "<< n->z<<")"<<endl<<flush;
        cout<<"Sc: ("<<sphereCenter->x<<", "<< sphereCenter->z<<")"<<endl<<flush;*/
        bool found = false;
        CircularList<Point*>::iterator cit = contour.at(boundary[i]);
        cit++;
        do{
            double dist = (**cit - *sphereCenter).length();
            if(dist < ray - EPSILON)
                found = true;
            cit++;
        }while(cit != contour.at(p));

        if(!found)
            newBoundary.push_back(p);

    }

    boundary = newBoundary;
    boundary.push_back(boundary[0]);

}

std::vector<Point *> Utilities::getLevelSet(std::map<Point *, double> morseFunction, double value)
{
    std::vector<Point *> levelSet;
    for(map<Point *, double>::iterator fit = morseFunction.begin(); fit != morseFunction.end(); fit++){
        if((*fit).second == value)
            levelSet.push_back((*fit).first);
    }

    return levelSet;
}


void Utilities::findFaces(std::vector<IMATI_STL::Triangle*> &faces, std::vector<IMATI_STL::Vertex*> vertices){

    //For each vertex, we find the triangle ring and we insert in the array all the faces that aren't in it
    for(unsigned long i = 0; i < vertices.size(); i++){

        for(IMATI_STL::Node* n = vertices[i]->VT()->head(); n != nullptr; n = n->next()){
            IMATI_STL::Triangle* t = static_cast<IMATI_STL::Triangle*>(n->data);

            vector<IMATI_STL::Triangle*>::iterator it = find(faces.begin(), faces.end(), t);
            if(it == faces.end())
                faces.push_back(t);
        }
    }
}

long double length(Point a, Point b){
    long double ax = a.x, ay = a.y, az = a.z, bx = b.x, by = b.y, bz = b.z;
    return sqrt((ax - bx) * (ax - bx) + (ay - by) * (ay - by) + (az - bz) * (az - bz));
}

bool Utilities::isPointInSegment(Point p, Point a, Point b)
{
    double l = (b - a).length();
    double l1 = (p - a).length();
    double l2 = (b - p).length();
    double w1 = l1 / l;
    double w2 = l2 / l;

    return w1 + w2 <= 1.0 + 1e-5;
}


bool Utilities::isPointInSegment(Vector3d p, Vector3d a, Vector3d b)
{
    double l = (b - a).norm();
    double l1 = (p - a).norm();
    double l2 = (b - p).norm();
    double w1 = l1 / l;
    double w2 = l2 / l;

    return w1 + w2 <= 1.0 + 1e-5;
}

std::pair<Point*, Point*> Utilities::compute2OrthogonalVersors(Point v)
{
    Point* v1 = new Point();
    double value1 = rand(), value2 = rand();
    if(abs(v.x) > 1e-6)
    {
        v1->x = (-value1 * v.y - value2 * v.z) / v.x;
        v1->y = value1;
        v1->z = value2;
    } else if (abs(v.y) > 1e-6)
    {
        v1->x = value1;
        v1->y = (-value1 * v.x - value2 * v.z) / v.y;
        v1->z = value2;
    } else if (abs(v.z) > 1e-6)
    {
        v1->x = value1;
        v1->y = value2;
        v1->z = (-value1 * v.x - value2 * v.y) / v.z;
    } else
    {
        v1->x = rand();
        v1->y = value1;
        v1->z = value2;
    }

    v1->normalize();

    Point* v2 = new Point(v & (*v1));
    v2->normalize();

    return make_pair(v1, v2);
}

Point* Utilities::edgePlaneIntersection(Edge e, Point normal, Point origin)
{

    std::pair<Point*, Point*> versors = compute2OrthogonalVersors(normal);
    Point p1 = origin + (*versors.first);
    Point p2 = origin + (*versors.second);
    double r1 = e.v1->exactOrientation(&origin, &p1, &p2);
    double r2 = e.v2->exactOrientation(&origin, &p1, &p2);
    Point* intersection = new Point(INFINITE_POINT);
    if(r1 * r2 < 0){
        intersection = new Vertex(Point::linePlaneIntersection(e.v1, e.v2, origin, p1, p2));
    }else if(r1 != r2){
        if(r1 == 0)
            intersection = e.v1;
        if(r2 == 0)
            intersection = e.v2;
    }else if (r1 == 0 && r2 == 0)
        intersection = e.v1;

    return intersection;
}


double normalizeValue(double value, double min, double max){
    return (value - min) / (max - min);
}


bool Utilities::areBoundariesEqual(std::vector<Point *> a, std::vector<Point *> b){
    unsigned int correspondencesNumber = 0;
    for(unsigned int i = 0; i < a.size(); i++)
        for(unsigned int j = 0; j < b.size(); j++)
            if(*a[i] == *b[j])
                correspondencesNumber++;
    return correspondencesNumber == a.size() || correspondencesNumber == b.size();
}


std::pair<Vector2d, double> Utilities::circleFrom3Points(Vector2d p1, Vector2d p2, Vector2d p3)
{
    Matrix3d A;
    A <<    p1.x(), p1.y(), 1,
            p2.x(), p2.y(), 1,
            p3.x(), p3.y(), 1;
    Vector3d b;
    b <<    -pow(p1.x(), 2) - pow(p1.y(), 2),
            -pow(p2.x(), 2) - pow(p2.y(), 2),
            -pow(p3.x(), 2) - pow(p3.y(), 2);

    Vector3d x = A.colPivHouseholderQr().solve(b);

    Vector2d center = {-x.x() / 2, -x.y() / 2};
    double ray = 0.5 * sqrt(pow(x.x(), 2) + pow(x.y(), 2) - 4 * x.z());

    return make_pair(center, ray);
}

std::vector<Vector2d> Utilities::lineCircleIntersection(Vector2d p1, Vector2d p2, Vector2d center, double ray)
{
    vector<Vector2d> intersections;
    double a = pow(p1.x() - p2.x(), 2) + pow(p1.y() - p2.y(), 2);
    double b = 2 * ((p1.x() - p2.x()) * (p2.x() - center.x()) + (p1.y() - p2.y()) * (p2.y() - center.y()));
    double c = pow(p2.x() - center.x(), 2) + pow(p2.y() - center.y(), 2) - pow(ray, 2);
    double delta = pow(b, 2) - 4 * a * c;
    double t1 = -1, t2 = -1;
    if(delta >= 0){
        t1 = (-b + sqrt(delta)) / (2 * a);
        if(delta > 0)
            t2 = (-b - sqrt(delta)) / (2 * a);
    }

    Vector2d i1, i2;
    if(t1 > 0 - ZERO_EPSILON && t1 < 1 + ZERO_EPSILON){
        i1 << p1.x() * t1 + p2.x() * (1 - t1), p1.y() * t1 + p2.y() * (1 - t1);
        intersections.push_back(i1);
    }

    if(t2 > 0 - ZERO_EPSILON && t2 < 1 + ZERO_EPSILON){
        i2 << p1.x() * t2 + p2.x() * (1 - t2), p1.y() * t2 + p2.y() * (1 - t2);
        intersections.push_back(i2);
    }

    return intersections;

}

double Utilities::computeLocalCurvature2D(Vertex *v, double circleRay, unsigned short neighbourhoodSize)
{
    double curvature = 0;

    IMATI_STL::Vertex* prev = v->prevOnBoundary(), * next = v->nextOnBoundary();
    for(unsigned int j = 1; j <= neighbourhoodSize; j++){
        Eigen::Vector2d p1, p1_, p2, p2_, c;
        c << v->x, v->z;
        while((*v - *prev).length() < circleRay * j)
            prev = prev->prevOnBoundary();
        while((*v - *next).length() < circleRay * j)
            next = next->nextOnBoundary();

        p1 << prev->x, prev->z;
        p1_ << prev->nextOnBoundary()->x, prev->nextOnBoundary()->z;
        p2 << next->x, next->z;
        p2_ << next->prevOnBoundary()->x, next->prevOnBoundary()->z;
        std::vector<Eigen::Vector2d> intersectionsPrev = lineCircleIntersection(p1, p1_, c, circleRay * j);
        std::vector<Eigen::Vector2d> intersectionsNext = lineCircleIntersection(p2, p2_, c, circleRay * j);
        double angle = std::acos((intersectionsNext[0] - c).dot(intersectionsPrev[0] - c) / ((intersectionsNext[0] - c).norm() * (intersectionsPrev[0] - c).norm()));
        if(isLeft(p2, p1, c))
            angle = - angle;

        curvature += angle;
    }

    curvature /= neighbourhoodSize;

    return curvature;
}

std::map<Vertex *, double> Utilities::computePolygonCurvature(ExtendedTrimesh *polygon)
{
    double circleRay = polygon->getMinEdgeLength();
    map<Vertex*, double> pointsCurvature;
    for (IMATI_STL::Node* n = polygon->V.head(); n != nullptr; n = n->next())
        pointsCurvature.insert(make_pair(static_cast<Vertex*>(n->data),
                               computeLocalCurvature2D(static_cast<Vertex*>(n->data), circleRay, LOCAL_BOUNDARY_DIM)));

    return pointsCurvature;
}

ExtendedTrimesh *Utilities::segmentSelection(ExtendedTrimesh *mesh, std::vector<std::vector<IMATI_STL::Vertex*> > selections)
{

    vector<Point*> boundary;
    ExtendedTrimesh* headMesh(mesh);

    int MARKED = 193;
    vector<IMATI_STL::Triangle*> seeds;
    IMATI_STL::Triangle* seed = nullptr;
    vector<IMATI_STL::Triangle*> toRemoveTriangles;
    for (unsigned int i = 0; i < selections.size(); i++) {
        vector<Vertex*> selection = selections[i];
        for (unsigned int j = 1; j < selection.size(); j++) {
            Edge* e = selection[j - 1]->getEdge(selection[j]);
            e->info = &MARKED;
            IMATI_STL::Triangle* t = e->rightTriangle(selection[j - 1]);

            vector<IMATI_STL::Triangle*>::iterator it = find(toRemoveTriangles.begin(), toRemoveTriangles.end(), t);
            if(t != nullptr && it == toRemoveTriangles.end()){
                Edge* te = t->e1;
                for(unsigned int k = 0; k < 3; k++){
                    if(te != nullptr && (te->info == nullptr || *static_cast<int*>(te->info) != MARKED))
                        seeds.push_back(te->oppositeTriangle(t));
                    te = t->nextEdge(te);
                }
                toRemoveTriangles.push_back(t);
            }
        }
    }

    unsigned int pos = 0;

    for (unsigned int i = 0; i < toRemoveTriangles.size(); i++) {
        seed = seeds[pos];
        if(seed == toRemoveTriangles[i])
            pos++;
        headMesh->removeTriangle(toRemoveTriangles[i]);
    }

    int selected = headMesh->selectConnectedComponent(seed);
    headMesh->removeSelectedTriangles();
    headMesh->save("Testa.ply");

    return headMesh;

}

ExtendedTrimesh *Utilities::generateCage(ExtendedTrimesh *shape, Utilities::type type)
{

    ExtendedTrimesh* cage = new ExtendedTrimesh(shape);
    cage->simplify(100, 1, 0, 3);
    if(type == Utilities::VOLUMETRIC){
        double offsetSize = shape->getMinEdgeLength() * 50;
        for(IMATI_STL::Node* n = cage->V.head(); n != nullptr; n = n->next()){
            Vertex* v = static_cast<Vertex*>(n->data);
            Point normal = v->getNormal();
            v->setValue(*v + normal * offsetSize);
        }
    }
    return cage;
//    ExtendedTrimesh* cage = new ExtendedTrimesh(shape);

//    simplifyMeshSemantically(cage);
//    cage->setIsCage(true);
//    cage->clearAnnotations();
//    return cage;


}


std::pair<Eigen::Vector3d, Eigen::Vector3d> Utilities::planarRegression(Matrix3Xd points)
{
    Vector3d centroid;
    centroid << 0.0, 0.0, 0.0;
    for (unsigned int i = 0; i < points.cols(); i++)
        centroid += points.col(i);
    centroid /= points.cols();
    for (unsigned int i = 0; i < points.cols(); i++)
        points.col(i) -= centroid;
    Matrix3d U = points.bdcSvd(ComputeFullU | ComputeThinV).matrixU();
    return std::make_pair(centroid, U.col(2));
}

void Utilities::simplifyMeshSemantically(ExtendedTrimesh *shape)
{
    std::vector<Annotation*> annotations = shape->getAnnotations();
    int criticalPointsNumber = 0;
    for(unsigned int i = 0; i < annotations.size(); i++){

        AnnotationType type = annotations[i]->getType();
        switch(type){

        case AnnotationType::Volume:
                break;

        case AnnotationType::Surface:
        {
            vector<vector<Vertex*> > outlines = dynamic_cast<SurfaceAnnotation*>(annotations[i])->getOutlines();
            for(unsigned int j = 0; j < outlines.size(); j++)
                for(unsigned int k = 0; k < outlines[j].size(); k++){
                    if(outlines[j][k]->generic_flag != CRITICAL_POINT)
                        criticalPointsNumber++;
                    outlines[j][k]->generic_flag = CRITICAL_POINT;
                }
            break;
        }

        case AnnotationType::Line:
        {
            vector<vector<Vertex*> > polylines = dynamic_cast<LineAnnotation*>(annotations[i])->getPolyLines();
            for(unsigned int j = 0; j < polylines.size(); j++)
                for(unsigned int k = 0; k < polylines[j].size(); k++){
                    if(polylines[j][k]->generic_flag != CRITICAL_POINT)
                        criticalPointsNumber++;
                    polylines[j][k]->generic_flag = CRITICAL_POINT;
                }
            break;
        }

        case AnnotationType::Point:
        {
            vector<Vertex*> points = dynamic_cast<PointAnnotation*>(annotations[i])->getPoints();
            for(unsigned int j = 0; j < points.size(); j++){
                if(points[j]->generic_flag != CRITICAL_POINT)
                    criticalPointsNumber++;
                points[j]->generic_flag = CRITICAL_POINT;
            }
            break;
        }

        }

    }

    int verticesNumber = criticalPointsNumber > 100 ? criticalPointsNumber : 100;
    shape->simplify(verticesNumber, 0, 0, 0);

}


std::vector<unsigned long> Utilities::verticesToIDvector(ExtendedTrimesh* mesh, std::vector<IMATI_STL::Vertex *> vertices)
{
    std::vector<unsigned long> verticesIds;
    for(unsigned int i = 0; i < vertices.size(); i++)
        verticesIds.push_back(mesh->getPointId(vertices[i]));

    return verticesIds;

}

std::vector<unsigned long> Utilities::trianglesToIDvector(ExtendedTrimesh* mesh, std::vector<IMATI_STL::Triangle *> triangles)
{
    std::vector<unsigned long> trianglesIds;
    for(unsigned int i = 0; i < triangles.size(); i++)
        trianglesIds.push_back(mesh->getTriangleId(triangles[i]));

    return trianglesIds;

}

std::vector<int> Utilities::verticesToIntIDvector(ExtendedTrimesh *mesh, std::vector<T_MESH::Vertex *> vertices)
{
    std::vector<int> verticesIds;
    for(unsigned int i = 0; i < vertices.size(); i++)
        verticesIds.push_back(mesh->getPointId(vertices[i]));

    return verticesIds;
}

void Utilities::polyLineFitting(std::vector<IMATI_STL::Vertex*> p1, std::vector<IMATI_STL::Vertex*> p2, int s1, int e1, int s2, int e2, std::map<unsigned int, unsigned int> &correspondences)
{
    if(s1 == 0 && e1 == p1.size() - 1 && s2 == 0 && e2 == p2.size() - 1)
    {
        correspondences.insert(std::make_pair(0, 0));
        correspondences.insert(std::make_pair(e1, e2));
    }

    if(e1 <= s1 + 1 || e2 <= s2 + 1)
        return;

    unsigned int i;
    unsigned int best_pos = 0;
    double l1 = 0.0, l1_ = 0.0;
    double l2 = 0.0, l2_ = 0.0;
    double best_distance = DBL_MAX;
    std::pair<unsigned int, unsigned int> correspondence;


    for(i = s1 + 1; i <= e1; i++)
        l1 += ((*p1[i]) - (*p1[i - 1])).length();
    l1 /= 2;

    for(i = s2 + 1; i <= e2; i++)
        l2 += ((*p2[i]) - (*p2[i - 1])).length();
    l2 /= 2;

    i = s1 + 1;
    while(l1_ < l1 && i <= e1)
    {
        l1_ += ((*p1[i]) - (*p1[i - 1])).length();
        double distance = abs(l1_ - l1);
        if(distance < best_distance){
            best_distance = distance;
            best_pos = i;
        }
        i++;
    }

    correspondence.first = best_pos;
    std::cout << "best_pos: " << best_pos << " l1: " << l1 << " l1_: " << l1_ << std::endl;
    best_distance = DBL_MAX;
    best_pos = 0;
    i = s2 + 1;
    while(l2_ < l2 && i <= e2)
    {
        l2_ += ((*p2[i]) - (*p2[i - 1])).length();
        double distance = abs(l2_ - l2);
        if(distance < best_distance){
            best_distance = distance;
            best_pos = i;
        }
        i++;
    }

    correspondence.second = best_pos;
    std::cout << "best_pos: " << best_pos << " l2: " << l2 << " l2_: " << l2_ << std::endl;

    correspondences.insert(correspondence);

    polyLineFitting(p1, p2, s1, correspondence.first, s2, correspondence.second, correspondences);
    polyLineFitting(p1, p2, correspondence.first, e1, correspondence.second, e2, correspondences);

}


bool isPointInSegment(Vector3d a, Vector3d b, Vector3d p)
{
    double l = (b - a).norm();
    double l1 = (p - a).norm();
    double l2 = (b - p).norm();
    double w1 = l1 / l;
    double w2 = l2 / l;

    return w1 + w2 <= 1.0 + 1e-5;
}

std::pair<unsigned int, std::vector<Point*>> Utilities::sphere_line_intersection (Point* p1, Point* p2, Point* p3, double r, bool inSegment)
{

    std::vector<Point*> intersections;
    unsigned int intersections_number = 0;
    double a, b, c, mu, i, epsilon = 1e-5;

    a =  pow(p2->x - p1->x, 2) + pow(p2->y - p1->y, 2) + pow(p2->z - p1->z, 2);
    b =  2 * ((p2->x - p1->x) * (p1->x - p3->x) +
              (p2->y - p1->y) * (p1->y - p3->y) +
              (p2->z - p1->z) * (p1->z - p3->z));
    c = pow(p3->x, 2) + pow(p3->y, 2) + pow(p3->z, 2) + pow(p1->x, 2) +
        pow(p1->y, 2) + pow(p1->z, 2) - 2 * (p3->x * p1->x + p3->y * p1->y + p3->z * p1->z) - pow(r, 2);
    i = pow(b, 2) - 4 * a * c;
    if ( i < 0.0 - epsilon)
        return(std::make_pair(intersections_number, intersections));

    mu = (-b + sqrt(i)) / (2 * a) ;
    IMATI_STL::Point* intersection = new IMATI_STL::Point( p1->x + mu * (p2->x - p1->x),
                             p1->y + mu * (p2->y - p1->y),
                             p1->z + mu * (p2->z - p1->z));
    if(!inSegment || isPointInSegment(*intersection, *p1, *p2))
    {
        intersections_number++;
        intersections.push_back(intersection);
    }

    if ( i > 0.0 + epsilon)
    {
        mu = (-b - sqrt(i)) / (2 * a);

        IMATI_STL::Point* intersection2 = new IMATI_STL::Point(p1->x + mu * (p2->x - p1->x),
                                 p1->y + mu * (p2->y - p1->y),
                                 p1->z + mu * (p2->z - p1->z));
        if(!inSegment || isPointInSegment(*intersection2, *p1, *p2))
        {
            intersections_number++;
            intersections.push_back(intersection2);
        }
    }

    return std::make_pair(intersections_number, intersections);

}


std::pair<unsigned int, std::vector<Vector3d>> sphere_line_intersection (Vector3d p1, Vector3d p2, Vector3d p3, double r, bool inSegment)
{


    /*std::cout << "Intersection between sphere centered in (" << p3.x() << "," << p3.y() << "," << p3.z() << ") with radius " << r <<
                 " and the line between (" << p1.x() << "," << p1.y() << "," << p1.z() << ") and (" << p2.x() << "," << p2.y() << "," << p2.z() << ")" << std::endl;*/

    std::vector<Vector3d> intersections;
    unsigned int intersections_number = 0;
    double a, b, c, mu, i, epsilon = 1e-5;

    a =  pow(p2.x() - p1.x(), 2) + pow(p2.y() - p1.y(), 2) + pow(p2.z() - p1.z(), 2);
    b =  2 * ((p2.x() - p1.x()) * (p1.x() - p3.x()) +
              (p2.y() - p1.y()) * (p1.y() - p3.y()) +
              (p2.z() - p1.z()) * (p1.z() - p3.z()));
    c = pow(p3.x(), 2) + pow(p3.y(), 2) + pow(p3.z(), 2) + pow(p1.x(), 2) +
        pow(p1.y(), 2) + pow(p1.z(), 2) - 2 * (p3.x() * p1.x() + p3.y() * p1.y() + p3.z() * p1.z()) - pow(r, 2);
    i = pow(b, 2) - 4 * a * c;
    if ( i < 0.0 - epsilon)
        return(std::make_pair(intersections_number, intersections));

    mu = (-b + sqrt(i)) / (2 * a) ;
    Vector3d intersection = { p1.x() + mu * (p2.x() - p1.x()),
                             p1.y() + mu * (p2.y() - p1.y()),
                             p1.z() + mu * (p2.z() - p1.z())};
    if(!inSegment || isPointInSegment(p1, p2, intersection))
    {
        intersections_number++;
        intersections.push_back(intersection);
    }

    if ( i > 0.0 + epsilon)
    {
        mu = (-b - sqrt(i)) / (2 * a);

        Vector3d intersection2 = {p1.x() + mu * (p2.x() - p1.x()),
                                 p1.y() + mu * (p2.y() - p1.y()),
                                 p1.z() + mu * (p2.z() - p1.z())};
        if(!inSegment || isPointInSegment(p1, p2, intersection2))
        {
            intersections_number++;
            intersections.push_back(intersection2);
        }
    }
    return std::make_pair(intersections_number, intersections);

}

//std::vector<Vector3d> sphere_triangulation_intersection(Vector3d center, double radius, std::vector<unsigned int> triangles, ExtendedTrimesh* mesh)
//{
//    int EDGE_FLAGGED = 9087;
//    int FLAGGED = 981;
//    std::vector<Vector3d> intersectionLoop;
//    std::vector<IMATI_STL::Edge*> usedEdges;
//    std::pair<unsigned int, std::vector<Vector3d> > intersections;
//    IMATI_STL::Edge* startingEdge = nullptr;
//    unsigned int i = 0;
//    while (startingEdge == nullptr && i < triangles.size() ) {
//        IMATI_STL::Triangle* t = mesh->getTriangle(triangles[i++]);
//        if(t->info != nullptr && *static_cast<int*>(t->info) == FLAGGED)
//            continue;

//        IMATI_STL::Vertex* v1 = t->v1();
//        for(unsigned int j = 0; j < 3; j++){
//            IMATI_STL::Vertex* v2 = t->nextVertex(v1);
//            if(v1->getEdge(v2)->info != nullptr && *static_cast<int*>(v1->getEdge(v2)->info) == EDGE_FLAGGED){
//                v1 = v2;
//                continue;
//            }
//            v1->getEdge(v2)->info = &EDGE_FLAGGED;
//            usedEdges.push_back(v1->getEdge(v2));
//            Vector3d p1 = {v1->x, v1->y, v1->z};
//            Vector3d p2 = {v2->x, v2->y, v2->z};
//            intersections = sphere_line_intersection(p1, p2, center, radius, true);
//            if(intersections.first > 0){
//                intersectionLoop.push_back(intersections.second[0]);
//                if(intersections.first > 1 && (intersections.second[1] - p1).norm() < (intersections.second[0] - p1).norm()){
//                    intersectionLoop.insert(intersectionLoop.begin(), intersections.second[1]);
//                }else if(intersections.first > 1){
//                    intersectionLoop.push_back(intersections.second[1]);
//                }
//                startingEdge = v1->getEdge(v2);

//            }
//        }

//    }

//    for(unsigned int i = 0; i < usedEdges.size(); i++)
//        usedEdges[i]->info = nullptr;

//    startingEdge->info = &EDGE_FLAGGED;
//    IMATI_STL::Edge* currentEdge;
//    IMATI_STL::Vertex* startingVertex = startingEdge->v1, * v1 = startingEdge->v2;
//    std::stack<std::pair<IMATI_STL::Vertex*, std::pair<IMATI_STL::Edge*, std::pair<unsigned int, std::vector<Vector3d> > > > > intersectedEdges;


//    do{
//        Vector3d p1 = {v1->x, v1->y, v1->z};
//        bool found = false;
//        currentEdge = nullptr;
//        for(IMATI_STL::Node* n = v1->VV()->head(); n != nullptr; n = n->next()){

//            IMATI_STL::Vertex* v2 = static_cast<IMATI_STL::Vertex*>(n->data);
//            if(v1->getEdge(v2)->info != nullptr && *static_cast<int*>(v1->getEdge(v2)->info) == EDGE_FLAGGED )
//                continue;
//            Vector3d p2 = {v2->x, v2->y, v2->z};
//            intersections = sphere_line_intersection(p1, p2, center, radius, true);
//            switch(intersections.first){
//            case 0:
//            {
//                continue;
//            }
//            case 1:
//            {
//                found = true;
//                intersectedEdges.push(std::make_pair(v1, std::make_pair(v1->getEdge(v2), intersections)));
//                v1->getEdge(v2)->info = &EDGE_FLAGGED;
//                break;
//            }
//            case 2:
//            {
//                found = true;
//                intersectedEdges.push(std::make_pair(v1, std::make_pair(v1->getEdge(v2), intersections)));
//                v1->getEdge(v2)->info = &EDGE_FLAGGED;
//                break;
//            }
//            }
//        }

//        if(intersectedEdges.size() == 0){
//            break;
//        }

//        v1 = intersectedEdges.top().first;

//        currentEdge = intersectedEdges.top().second.first;
//        intersections = intersectedEdges.top().second.second;
//        intersectedEdges.pop();
//        currentEdge->t1->info = &FLAGGED;
//        currentEdge->t2->info = &FLAGGED;
//        intersectionLoop.push_back(intersections.second[0]);
//        if(intersections.first > 1 && (intersections.second[1] - p1).norm() < (intersections.second[0] - p1).norm())
//            intersectionLoop.insert(intersectionLoop.begin(), intersections.second[1]);
//        else if(intersections.first > 1)
//            intersectionLoop.push_back(intersections.second[1]);
//        v1 = currentEdge->oppositeVertex(v1);
//    } while (v1 != startingVertex);


//    for(unsigned int i = 0; i < triangles.size(); i++)
//    {
//        IMATI_STL::Triangle* t = mesh->getTriangle(triangles[i]);
//        t->v1()->getEdge(t->v2())->info = nullptr;
//        t->v2()->getEdge(t->v3())->info = nullptr;
//        t->v3()->getEdge(t->v1())->info = nullptr;
//    }

//    return intersectionLoop;
//}


//std::vector<Vector3d> sphere_triangulation_intersections(Vector3d center, double radius, std::vector<unsigned int> triangles, ExtendedTrimesh* mesh)
//{
//    int EDGE_FLAGGED = 9087;
//    int FLAGGED = 981;
//    std::vector<Vector3d> intersectionLoop;
//    std::vector<IMATI_STL::Edge*> usedEdges;
//    std::pair<unsigned int, std::vector<Vector3d> > intersections;
//    IMATI_STL::Edge* startingEdge = nullptr;
//    unsigned int i = 0;
//    while (startingEdge == nullptr && i < triangles.size() ) {
//        IMATI_STL::Triangle* t = mesh->getTriangle(triangles[i++]);
//        if(t->info != nullptr && *static_cast<int*>(t->info) == FLAGGED)
//            continue;

//        IMATI_STL::Vertex* v1 = t->v1();
//        for(unsigned int j = 0; j < 3; j++){
//            IMATI_STL::Vertex* v2 = t->nextVertex(v1);
//            if(v1->getEdge(v2)->info != nullptr && *static_cast<int*>(v1->getEdge(v2)->info) == EDGE_FLAGGED){
//                v1 = v2;
//                continue;
//            }
//            v1->getEdge(v2)->info = &EDGE_FLAGGED;
//            usedEdges.push_back(v1->getEdge(v2));
//            Vector3d p1 = {v1->x, v1->y, v1->z};
//            Vector3d p2 = {v2->x, v2->y, v2->z};
//            intersections = sphere_line_intersection(p1, p2, center, radius, true);
//            if(intersections.first > 0){
//                intersectionLoop.push_back(intersections.second[0]);
//                if(intersections.first > 1 && (intersections.second[1] - p1).norm() < (intersections.second[0] - p1).norm()){
//                    intersectionLoop.insert(intersectionLoop.begin(), intersections.second[1]);
//                }else if(intersections.first > 1){
//                    intersectionLoop.push_back(intersections.second[1]);
//                }
//                startingEdge = v1->getEdge(v2);

//            }
//        }

//    }

//    for(unsigned int i = 0; i < usedEdges.size(); i++)
//        usedEdges[i]->info = nullptr;

//    startingEdge->info = &EDGE_FLAGGED;
//    IMATI_STL::Edge* currentEdge;
//    IMATI_STL::Vertex* startingVertex = startingEdge->v1, * v1 = startingEdge->v2;
//    std::stack<std::pair<IMATI_STL::Vertex*, std::pair<IMATI_STL::Edge*, std::pair<unsigned int, std::vector<Vector3d> > > > > intersectedEdges;


//    do{
//        Vector3d p1 = {v1->x, v1->y, v1->z};
//        bool found = false;
//        currentEdge = nullptr;
//        for(IMATI_STL::Node* n = v1->VV()->head(); n != nullptr; n = n->next()){

//            IMATI_STL::Vertex* v2 = static_cast<IMATI_STL::Vertex*>(n->data);
//            if(v1->getEdge(v2)->info != nullptr && *static_cast<int*>(v1->getEdge(v2)->info) == EDGE_FLAGGED )
//                continue;
//            Vector3d p2 = {v2->x, v2->y, v2->z};
//            intersections = sphere_line_intersection(p1, p2, center, radius, true);
//            switch(intersections.first){
//            case 0:
//            {
//                continue;
//            }
//            case 1:
//            {
//                found = true;
//                intersectedEdges.push(std::make_pair(v1, std::make_pair(v1->getEdge(v2), intersections)));
//                v1->getEdge(v2)->info = &EDGE_FLAGGED;
//                break;
//            }
//            case 2:
//            {
//                found = true;
//                intersectedEdges.push(std::make_pair(v1, std::make_pair(v1->getEdge(v2), intersections)));
//                v1->getEdge(v2)->info = &EDGE_FLAGGED;
//                break;
//            }
//            }
//        }

//        if(intersectedEdges.size() == 0){
//            break;
//        }

//        v1 = intersectedEdges.top().first;

//        currentEdge = intersectedEdges.top().second.first;
//        intersections = intersectedEdges.top().second.second;
//        intersectedEdges.pop();
//        currentEdge->t1->info = &FLAGGED;
//        currentEdge->t2->info = &FLAGGED;
//        intersectionLoop.push_back(intersections.second[0]);
//        if(intersections.first > 1 && (intersections.second[1] - p1).norm() < (intersections.second[0] - p1).norm())
//            intersectionLoop.insert(intersectionLoop.begin(), intersections.second[1]);
//        else if(intersections.first > 1)
//            intersectionLoop.push_back(intersections.second[1]);
//        v1 = currentEdge->oppositeVertex(v1);
//    } while (v1 != startingVertex);


//    for(unsigned int i = 0; i < triangles.size(); i++)
//    {
//        IMATI_STL::Triangle* t = mesh->getTriangle(triangles[i]);
//        t->v1()->getEdge(t->v2())->info = nullptr;
//        t->v2()->getEdge(t->v3())->info = nullptr;
//        t->v3()->getEdge(t->v1())->info = nullptr;
//    }

//    return intersectionLoop;
//}

//MatrixX3d Utilities::extractSkeleton(ExtendedTrimesh* mesh, std::vector<unsigned int> triangles, std::vector<Vector3d> seedLoop)
//{
//    MatrixX3d skeleton;
//    std::vector<Vector3d> nodes;
//    for(unsigned int i = 0; i < triangles.size(); i++)
//        mesh->getTriangle(triangles[i])->info = nullptr;

//    std::vector<Vector3d> loop = seedLoop;
//    Vector3d node;
//    do{
//        std::cout << "New loop:" << std::endl;
//        node.setZero();

//        for(unsigned int i = 0; i < loop.size(); i++){
//            node += loop[i];
//            std::cout << loop[i].x() << " "  << loop[i].y() << " "  << loop[i].z() << std::endl;
//        }

//        std::cout<<std::flush;
//        node /= loop.size();
//        nodes.push_back(node);
//        std::cout << "Center: (" << node.x() << "," << node.y() << "," << node.z() << ")" << std::endl;
//        double radius = 0.0;
//        for(unsigned int i = 0; i < loop.size(); i++){
//            double dist = (node - loop[i]).norm();
//            if(radius < dist)
//                radius = dist;
//        }

//        radius += radius / 2;
//        std::cout << "Radius: " << radius << std::endl;
//        loop = sphere_triangulation_intersection(node, radius, triangles, mesh);

//    } while(loop.size() != 0);

//    skeleton.resize(nodes.size(), 3);

//    for(unsigned int i = 0; i < nodes.size(); i++)
//        skeleton.row(i) = nodes[i];
//    for(unsigned int i = 0; i < triangles.size(); i++)
//        mesh->getTriangle(triangles[i])->info = nullptr;

//    return skeleton;
//}


Vector3d imatiToEigenPoint(IMATI_STL::Point v){
    Vector3d p = {v.x, v.y, v.z};
    return p;
}

//Eigen::MatrixX3d Utilities::extractSkeleton(ExtendedTrimesh* mesh, std::vector<unsigned int> triangles, std::vector<std::vector<unsigned int> > boundaries, unsigned int seedLoop)
//{
//    int EDGE_FLAGGED = 981;
//    int TRIANGLE_FLAGGED = 982;
//    int TRIANGLE_INSERTED = 983;
//    MatrixX3d skeleton;
//    std::vector<Vector3d> nodes;
//    for(unsigned int i = 0; i < triangles.size(); i++)
//        mesh->getTriangle(triangles[i])->info = nullptr;
//    std::vector<IMATI_STL::Triangle*> loopTriangles;
//    std::vector<IMATI_STL::Point*> intersectionPoints;
//    Vertex* v1 = mesh->getPoint(boundaries[seedLoop][0]);
//    for(unsigned int i = 1; i < boundaries[seedLoop].size(); i++){
//        Vertex* v2 = mesh->getPoint(boundaries[seedLoop][i]);
//        loopTriangles.push_back(v1->getEdge(v2)->leftTriangle(v1));
//        intersectionPoints.push_back(v1);
//        v1->getEdge(v2)->info = &EDGE_FLAGGED;
//        v1 = v2;
//    }

//    IMATI_STL::Point* node = new IMATI_STL::Point(0,0,0);
//    do{
//        node->setValue(0,0,0);
//        for(unsigned int i = 0; i < intersectionPoints.size(); i++){
//            *node += *(intersectionPoints[i]);
//        }

//        *node /= intersectionPoints.size();
//        nodes.push_back(imatiToEigenPoint(node));
//        double radius = 0.0;
//        for(unsigned int i = 0; i < intersectionPoints.size(); i++){
//            double dist = ((*node) - (*(intersectionPoints[i]))).length();
//            if(radius < dist)
//                radius = dist;
//        }

//        radius += radius / 2;

//        std::queue<IMATI_STL::Triangle*> Q;
//        for(unsigned int i = 0; i < loopTriangles.size(); i++){
//            loopTriangles[i]->info = &TRIANGLE_INSERTED;
//            Q.push(loopTriangles[i]);
//        }
//        intersectionPoints.clear();
//        loopTriangles.clear();
//        while(Q.size() > 0)
//        {
//            bool intersected = false;
//            IMATI_STL::Triangle* t = Q.front();
//            Q.pop();
//            if(t->info != nullptr && *static_cast<int*>(t->info) == TRIANGLE_FLAGGED)
//                continue;
//            t->info = &TRIANGLE_FLAGGED;
//            Edge* e = t->e1;
//            for(unsigned int i = 0; i < 3; i++){
//                std::pair<unsigned int, std::vector<Vector3d>> intersections = sphere_line_intersection(imatiToEigenPoint(e->v1), imatiToEigenPoint(e->v2), imatiToEigenPoint(node), radius, true);
//                if(intersections.first > 0)
//                    intersected = true;
//                else if(e->info == nullptr || *static_cast<int*>(e->info) != EDGE_FLAGGED) {
//                    IMATI_STL::Triangle* t_ = e->oppositeTriangle(t);
//                    if(!(t_->info != nullptr && (*static_cast<int*>(t_->info) == TRIANGLE_FLAGGED || *static_cast<int*>(t_->info) == TRIANGLE_INSERTED)))
//                    {
//                        t_->info = &TRIANGLE_INSERTED;
//                        Q.push(t_);
//                    }
//                }
//                e = t->nextEdge(e);
//            }
//            if(intersected)
//                loopTriangles.push_back(t); //No need to test presence: triangles are used only once
//        }
//    } while(loopTriangles.size() != 0);

//    skeleton.resize(nodes.size(), 3);

//    for(unsigned int i = 0; i < nodes.size(); i++)
//        skeleton.row(i) = nodes[i];
//    for(unsigned int i = 0; i < triangles.size(); i++)
//        mesh->getTriangle(triangles[i])->info = nullptr;

//    return skeleton;
//}


std::vector<unsigned int> pointsToTriangles(std::vector<unsigned int> points, ExtendedTrimesh* mesh)
{
    std::vector<unsigned int> triangles;
    int FLAGGED = 918;

    for(unsigned int i = 0; i < points.size(); i++){
        IMATI_STL::Vertex* p = mesh->getPoint(points[i]);
        IMATI_STL::List* vt = p->VT();
        for(IMATI_STL::Node* n = vt->head(); n != nullptr; n = n->next()){
            IMATI_STL::Triangle* t = static_cast<IMATI_STL::Triangle*>(n->data);
            if(t->info == nullptr || *static_cast<int*>(t->info) != FLAGGED){
                triangles.push_back(mesh->getTriangleId(t));
                t->info = &FLAGGED;
            }
        }
    }


    for(unsigned int i = 0; i < triangles.size(); i++)
        mesh->getTriangle(triangles[i])->info = nullptr;

    return triangles;
}

std::vector<int> Utilities::trianglesToPoints(std::vector<unsigned int> triangles, ExtendedTrimesh *mesh)
{
    std::vector<int> points;
    int FLAGGED = 918;

    for(unsigned int i = 0; i < triangles.size(); i++){
        IMATI_STL::Triangle* t = mesh->getTriangle(triangles[i]);
        IMATI_STL::Vertex* v = t->v1();
        for(unsigned int j = 0; j < 3; j++)
        {
            if(v->info == nullptr || *static_cast<int*>(v->info) != FLAGGED)
            {
                points.push_back(mesh->getPointId(v));
                v->info = &FLAGGED;
            }
            v = t->nextVertex(v);
        }
    }


    for(unsigned int i = 0; i < points.size(); i++)
        mesh->getTriangle(points[i])->info = nullptr;

    return points;
}

std::vector<unsigned int> Utilities::pointsToTriangles(std::vector<int> points, ExtendedTrimesh *mesh)
{
    std::vector<unsigned int> triangles;
    int FLAGGED = 918;

    for(unsigned int i = 0; i < points.size(); i++){
        IMATI_STL::Vertex* p = mesh->getPoint(points[i]);
        IMATI_STL::List* vt = p->VT();
        for(IMATI_STL::Node* n = vt->head(); n != nullptr; n = n->next()){
            IMATI_STL::Triangle* t = static_cast<IMATI_STL::Triangle*>(n->data);
            if(t->info == nullptr || *static_cast<int*>(t->info) != FLAGGED){
                triangles.push_back(mesh->getTriangleId(t));
                t->info = &FLAGGED;
            }
        }
    }


    for(unsigned int i = 0; i < triangles.size(); i++)
        mesh->getTriangle(triangles[i])->info = nullptr;

    return triangles;
}

bool isLarger(std::vector<Vertex*> a, std::vector<Vertex*> b)
{
    return a.size() > b.size();
}

std::vector<std::pair<IMATI_STL::Point*, std::vector<IMATI_STL::Vertex*> > > Utilities::extractSkeletonWithLoops(ExtendedTrimesh *mesh, std::vector<unsigned int> triangles, std::vector<std::vector<unsigned int> > boundaries, unsigned int seedLoop)
{
    int EDGE_FLAGGED = 981;
    int EDGE_USED_FLAG = 982;
    int TRIANGLE_FLAGGED = 983;
    int TRIANGLE_INSERTED = 984;
    MatrixX3d skeleton;
    std::vector<std::pair<IMATI_STL::Point*, std::vector<IMATI_STL::Vertex*> > > skeletonWithLoops;
    std::vector<Vector3d> nodes;
    for(unsigned int i = 0; i < triangles.size(); i++)
        mesh->getTriangle(triangles[i])->info = nullptr;
    std::vector<IMATI_STL::Triangle*> loopTriangles;
    std::vector<IMATI_STL::Triangle*> usedTriangles;
    std::vector<IMATI_STL::Vertex*> loop;
    std::vector<IMATI_STL::Point*> intersectionPoints;

    for(unsigned int i = 0; i < boundaries.size(); i++)
    {
        Vertex* v1 = mesh->getPoint(boundaries[i][0]);
        for(unsigned int j = 1; j < boundaries[i].size(); j++)
        {
            Vertex* v2 = mesh->getPoint(boundaries[i][j]);
            if(i == seedLoop)
            {
                loop.push_back(v1);
                loopTriangles.push_back(v1->getEdge(v2)->leftTriangle(v1));
                intersectionPoints.push_back(v1);
            }
            v1->getEdge(v2)->info = &EDGE_FLAGGED;
            v1 = v2;
        }
        if(i == seedLoop)
            loop.push_back(v1);
    }

    reverse(loop.begin(), loop.end());

    IMATI_STL::Point* node = new IMATI_STL::Point(0,0,0);
//    std::ofstream writer;
//    writer.open("cacchina.mat");
    int counter = 0;
    bool isBeginning = true;
//    if(writer.is_open())
//    {
        do{
            node->setValue(0,0,0);
            for(unsigned int i = 0; i < intersectionPoints.size(); i++){
                *node += *(intersectionPoints[i]);
            }

            *node /= intersectionPoints.size();
            IMATI_STL::Point* center = new IMATI_STL::Point(*node);
            skeletonWithLoops.push_back(make_pair(center, loop));
            nodes.push_back(imatiToEigenPoint(node));
            double radius = 0.0;
            for(unsigned int i = 0; i < intersectionPoints.size(); i++){
                double dist = ((*node) - (*(intersectionPoints[i]))).length();
                if(radius < dist)
                    radius = dist;
            }

            radius *= 2 ;

//            writer.put(char('a' + counter));
//            writer << "= [" << center->x << " " << center->y << " " << center->z << "]" << std::endl;
//            writer.put(char('A' + counter));
//            writer << "= [" << loop[0]->x << " " << loop[0]->y << " " << loop[0]->z << std::endl;
            std::queue<IMATI_STL::Triangle*> Q;
            for(unsigned int i = 1; i < loop.size(); i++)
            {
//                writer << loop[i]->x << " " << loop[i]->y << " " << loop[i]->z << std::endl;
                IMATI_STL::Edge* e = loop[i - 1]->getEdge(loop[i]);
                if(isBeginning || e->info == nullptr || *static_cast<int*>(e->info) != EDGE_FLAGGED )
                {
                    loop[i - 1]->getEdge(loop[i])->info = &EDGE_USED_FLAG;
                    Q.push(loop[i - 1]->getEdge(loop[i])->rightTriangle(loop[i - 1]));
                    Q.back()->info = &TRIANGLE_INSERTED;
                }
            }
//            writer << "];" << std::endl;

            if(isBeginning)
                isBeginning = false;

            intersectionPoints.clear();
            usedTriangles.clear();
            loopTriangles.clear();
            while(Q.size() > 0)
            {
                bool intersected = false;
                IMATI_STL::Triangle* t = Q.front();
                Q.pop();
                if(t->info != nullptr && *static_cast<int*>(t->info) == TRIANGLE_FLAGGED)
                    continue;
                usedTriangles.push_back(t);
                t->info = &TRIANGLE_FLAGGED;
                Edge* e = t->e1;
                for(unsigned int i = 0; i < 3; i++){
                    std::pair<unsigned int, std::vector<Point*> > intersections = sphere_line_intersection(e->v1, e->v2, node, radius, true);
                    if(intersections.first > 0)
                    {
                        intersectionPoints.push_back(intersections.second[0]);
                        if(intersections.first == 2)
                            intersectionPoints.push_back(intersections.second[1]);
                        intersected = true;
                    }
                    else if(e->info == nullptr || (*static_cast<int*>(e->info) != EDGE_FLAGGED && *static_cast<int*>(e->info) != EDGE_USED_FLAG)) {
                        IMATI_STL::Triangle* t_ = e->oppositeTriangle(t);
                        if(!(t_->info != nullptr && (*static_cast<int*>(t_->info) == TRIANGLE_FLAGGED || *static_cast<int*>(t_->info) == TRIANGLE_INSERTED)))
                        {
                            t_->info = &TRIANGLE_INSERTED;
                            Q.push(t_);
                        }
                    }
                    e = t->nextEdge(e);
                }
                if(intersected)
                    loopTriangles.push_back(t); //No need to test presence: triangles are used only once
            }

            std::vector<std::vector<Vertex*> > outlines = Utilities::computeOutlines(usedTriangles);
            while(outlines.size() > 2)
            {
                std::sort(outlines.begin(), outlines.end(), isLarger);
                outlines.erase(outlines.begin() + outlines.size());
            }
            for(unsigned int i = 0; i < outlines.size(); i++)
            {
                std::vector<Vertex*>::iterator it = find(outlines[i].begin(), outlines[i].end(), loop[0]);
                if(it == outlines[i].end())
                {
                    loop.clear();
                    loop = outlines[i];
                }
            }
            counter++;
        } while(loopTriangles.size() != 0);
//    }
    skeleton.resize(nodes.size(), 3);

//    writer.close();
    for(unsigned int i = 0; i < nodes.size(); i++)
        skeleton.row(i) = nodes[i];
    for(unsigned int i = 0; i < triangles.size(); i++)
        mesh->getTriangle(triangles[i])->info = nullptr;

    return skeletonWithLoops;
}

vector<vector<Vertex *> > Utilities::computeOutlines(vector<IMATI_STL::Triangle *> set){

    vector<std::pair<Vertex*, Vertex*> > setOutlineEdges;
    vector<vector<Vertex*> > outlines;
    Vertex* v, *v_;
    int IS_INSIDE = 738;

//    ofstream writer;
//    writer.open("megacacchina.mat");
    for(std::vector<IMATI_STL::Triangle*>::iterator tit = set.begin(); tit != set.end(); tit++)
    {
        std::pair<int,void*> *p = new std::pair<int,void*>(make_pair(IS_INSIDE, (*tit)->info));
        (*tit)->info = p;
        Vertex* v1 = (*tit)->v1();
        for(unsigned int i = 0; i < 3; i++)
        {
            Vertex* v2 = (*tit)->nextVertex(v1);
//            writer << v1->x << " " << v1->y << " " << v1->z << " " << v2->x << " " << v2->y << " " << v2->z << std::endl;
            v1 = v2;
        }
    }

//    writer.close();
    for(std::vector<IMATI_STL::Triangle*>::iterator tit = set.begin(); tit != set.end(); tit++){
        IMATI_STL::Triangle* t = static_cast<IMATI_STL::Triangle*>(*tit);
        Edge* e = t->e1;
        for(int i = 0; i < 3; i++){
            IMATI_STL::Triangle* t_ = e->oppositeTriangle(t);
            if(t_ == nullptr || t_->info == nullptr || static_cast<std::pair<int,void*> *>(t_->info)->first != IS_INSIDE){
                Edge* e_ = t->prevEdge(e);
                v = e_->commonVertex(e);
                setOutlineEdges.push_back(std::make_pair(v, e->oppositeVertex(v)));
            }
            e = t->nextEdge(e);
        }
    }

    while(setOutlineEdges.size() != 0){

        vector<Vertex*> outline;
        v = setOutlineEdges[0].first;
        Vertex *initialVertex = v;

        std::pair<Vertex*, Vertex*> pPrev = std::make_pair(nullptr, nullptr);
        do{
            outline.push_back(v);
            for(IMATI_STL::Node *n = v->VV()->head(); n != nullptr; n = n->next()){
                v_ = static_cast<Vertex*>(n->data);
                std::pair<Vertex*, Vertex*> p = std::make_pair(v, v_);
                for(vector<pair<Vertex*, Vertex*> >::iterator pit = setOutlineEdges.begin(); pit != setOutlineEdges.end(); pit++){
                    pair<Vertex*, Vertex*> tmp = static_cast<pair<Vertex*, Vertex*> >(*pit);
                    if(p != pPrev &&
                       ((p.first == tmp.second && p.second == tmp.first) ||
                       (p.first == tmp.second && p.second == tmp.first)) ){
                        v = v_;
                        pPrev = p;
                        setOutlineEdges.erase(pit);
                        break;
                    }
                }

                if(v == v_)
                    break;
            }
        }while(v != initialVertex);
        outline.push_back(outline[0]);
        outlines.push_back(outline);
    }

    for(vector<vector<Vertex*> >::iterator oit = outlines.begin(); oit != outlines.end(); oit++){
        vector<Vertex*> outline = static_cast<vector<Vertex*> >(*oit);
        IMATI_STL::Triangle* t = nullptr;
        for (unsigned int i = 1; i < outline.size(); i++) {
            t = outline[i - 1]->getEdge(outline[i])->leftTriangle(outline[i - 1]);
            if(t != nullptr)
                break;
        }
        if(t == nullptr || std::find(set.begin(), set.end(), t) == set.end())
            std::reverse(oit->begin(), oit->end());
    }

    for(std::vector<IMATI_STL::Triangle*>::iterator tit = set.begin(); tit != set.end(); tit++){
        (*tit)->info = static_cast<std::pair<int,void*> *>((*tit)->info)->second;
    }

    return outlines;

}

std::vector<unsigned int> Utilities::regionGrowing(ExtendedTrimesh *mesh, std::vector<std::vector<unsigned int> > boundaries)
{
    /* neighbors is a queue that stores the list of available triangles. A queue is a data structure
     * that follows the FIFO service order, see https://en.wikipedia.org/wiki/FIFO_(computing_and_electronics).
     * So, when selecting which triangle to use from the list of available triangles, you
     * always take the one that has been inside the queue for more time.
     */
    queue<IMATI_STL::Triangle*> neighbors;
    //List of triangles of the annotation (initially empty)
    vector<unsigned int> internalTriangles;
    /* Value that says when an edge is on the boundary (if the value in its "info" field is equal to 1,
     * then the edge is on the boundary). I don't know if you can add attributes to edges in Unity, so maybe it
     * is better if you just use the method with 3 lists that we used in the example
     */
    unsigned int BOUNDARY_EDGE = 1;
    /* Value that says when a triangle has already been used (if the value in its "info" field is equal to 1,
     * then the edge is on the boundary). Again, I don't know if you can add attributes to triangle in Unity, so
     * maybe it better if you just use the method with 3 lists that we used in the example
     */
    unsigned int ALREADY_USED = 9;

    //We iterate over the list of boundaries
    for(unsigned int i = 0; i < boundaries.size(); i++){

        //This is the i-th boundary
        vector<unsigned int> boundary = boundaries[i];
        /* For each boundary, we take the first two vertices (boundary[0] and boundary[1]) in the boundary.
         * The boundary is ordered counterclockwise, so you have to consider the triangle (t) on the left of
         * the edge connecting the two vertices.
         */
        IMATI_STL::Triangle* t = mesh->getPoint(boundary[0])->getEdge(mesh->getPoint(boundary[1]))->leftTriangle(mesh->getPoint(boundary[0]));
        internalTriangles.push_back(mesh->getTriangleId(t));
        t->info = static_cast<void*>(&ALREADY_USED);
        //We insert the triangle (or seed of the region growing algorithm) in the queue of available triangles
        neighbors.push(t);

        //We mark each edge in the boundary as a boundary edge
        for(unsigned int i = 1; i <= boundary.size() - 1; i++){

            Vertex* v1 = mesh->getPoint(boundary[i - 1]);
            Vertex* v2 = mesh->getPoint(boundary[i]);

            Edge* e = v1->getEdge(v2);
            if(e != nullptr)
                e->info = static_cast<void*>(&BOUNDARY_EDGE);
            else {
                std::cerr << "This shouldn't happen. Error in the identification of the edge." << std::endl << std::flush;
                exit(5);
            }
        }
    }

    //We will analyse triangles until the queue of available triangles is not empty.
    while(neighbors.size() > 0){
        //We take the first triangle (t) from the queue and remove it from there
        IMATI_STL::Triangle* t = neighbors.front();
        neighbors.pop();
        //We take the first edge (e) of t
        Edge* e = t->e1;
        //t has 3 edges, so we will perform the analysis 3 times
        for(int i = 0; i < 3; i++){
            //If e is not on the boundary
            if(e->info == nullptr || *static_cast<unsigned int*>(e->info) != BOUNDARY_EDGE){
                //We take the triangle (t_) on the opposite side of e with respect to t.
                IMATI_STL::Triangle* t_ = e->oppositeTriangle(t);
                if(t_ == nullptr)
                    continue;   //the mesh is not watertight and the edge is on the bounday of the mesh
                //If t_ has't been used yet, we insert it in the queue of usable triangles and in the list of triangles that are part of the annotation
                if(t_->info == nullptr ||  *static_cast<unsigned int*>(t_->info) != ALREADY_USED){
                    internalTriangles.push_back(mesh->getTriangleId(t_));
                    t_->info = static_cast<void*>(&ALREADY_USED);
                    neighbors.push(t_);
                }
            }
            //We take the next edge of t.
            e = t->nextEdge(e);
        }
    }

    //The actual algorithm is ended.

    //Ignore this for cycle, I am just removing the markings from the edges
    for(unsigned int i = 0; i < boundaries.size(); i++){

        vector<unsigned int> boundary = boundaries[i];
        for(unsigned int i = 1; i <= boundary.size() - 1; i++){

            Vertex* v1 = mesh->getPoint(boundary[i - 1]);
            Vertex* v2 = mesh->getPoint(boundary[i]);

            Edge* e = v1->getEdge(v2);
            if(e != nullptr)
                e->info = nullptr;
        }
    }


    //Ignore this for cycle, I am just removing the markings from the triangles
    for(std::vector<unsigned int>::iterator it = internalTriangles.begin(); it != internalTriangles.end(); it++)
        mesh->getTriangle(*it)->info = nullptr;


    //Now the list has all the triangles in the annotation.
    return internalTriangles;

}

double Utilities::computeMeshVolume(ExtendedTrimesh *mesh)
{
    double volume = 0.0;
    for(IMATI_STL::Node* n = mesh->T.head(); n != nullptr; n = n->next())
    {
        IMATI_STL::Triangle* t = static_cast<IMATI_STL::Triangle*>(n->data);
        volume += t->getNormal() * t->getCenter() * t->area();
    }
    volume /= 3;
    std::cout << "Volume: " << volume << std::endl << std::flush;
    return volume;
}
