#include "utilities.h"

#include <queue>
#include <stack>
#include <mutex>
#include <math.h>

#include <surfaceannotation.h>
#include <lineannotation.h>
#include <pointannotation.h>
#include <circularlist.h>

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


bool isPointInSegment(Vector3d a, Vector3d b, Vector3d p)
{
    double l = (b - a).norm();
    double l1 = (p - a).norm();
    double l2 = (b - p).norm();
    double w1 = l1 / l;
    double w2 = l2 / l;

    return w1 + w2 <= 1.0 + 1e-5;
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

std::vector<unsigned long> Utilities::trianglesToIDvector(ExtendedTrimesh* mesh, std::vector<IMATI_STL::Triangle *> triangles)
{
    std::vector<unsigned long> trianglesIds;
    for(unsigned int i = 0; i < triangles.size(); i++)
        trianglesIds.push_back(mesh->getTriangleId(triangles[i]));

    return trianglesIds;

}
