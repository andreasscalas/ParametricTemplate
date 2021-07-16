/****************************************************************************
* TMesh                                                                  *
*                                                                           *
* Consiglio Nazionale delle Ricerche                                        *
* Istituto di Matematica Applicata e Tecnologie Informatiche                *
* Sezione di Genova                                                         *
* IMATI-GE / CNR                                                            *
*                                                                           *
* Authors: Marco Attene                                                     *
* Copyright(C) 2013: IMATI-GE / CNR                                         *
* All rights reserved.                                                      *
*                                                                           *
* This program is dual-licensed as follows:                                 *
*                                                                           *
* (1) You may use TMesh as free software; you can redistribute it and/or *
* modify it under the terms of the GNU General Public License as published  *
* by the Free Software Foundation; either version 3 of the License, or      *
* (at your option) any later version.                                       *
* In this case the program is distributed in the hope that it will be       *
* useful, but WITHOUT ANY WARRANTY; without even the implied warranty of    *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU General Public License (http://www.gnu.org/licenses/gpl.txt)          *
* for more details.                                                         *
*                                                                           *
* (2) You may use TMesh as part of a commercial software. In this case a *
* proper agreement must be reached with the Authors and with IMATI-GE/CNR   *
* based on a proper licensing contract.                                     *
*                                                                           *
****************************************************************************/

#include "edge.h"
#include "triangle.h"

namespace T_MESH
{



//////////////////////// Constructor ///////////////////////

Edge::Edge(){
	mask = 0;
	info = NULL;
}
Edge::Edge(Vertex *va, Vertex *vb)
{
 v1 = va;
 v2 = vb;
 t1 = t2 = NULL;
 info = NULL;
 mask = 0;
}


////////////////// Destructor /////////////////////////////

Edge::~Edge()
{
}


////// Returns the unit vector for the edge direction //////

Point Edge::toUnitVector() const
{
 Point v = toVector();
 coord l = v.length();

 if (l == 0) TMesh::error("Edge::toUnitVector : Degenerate Edge !\n");

 return v/l;
}


/// Returns the edge normal.				////
/// It is the average of the incident triangle normals  ////

Point Edge::getNormal() const
{
 Point nor, n1, n2;
 
 if (t1 == NULL || t2 == NULL) return Point(0,0,0);

 n1 = t1->getNormal();
 n2 = t2->getNormal();
 nor = n1+n2;
 if (nor.length() != 0.0) nor.normalize();
 return nor;
}

////////////////////////// Edge swap ////////////////////////

bool Edge::swap(const bool fast)
{
 if (!fast && (t1 == NULL || t2 == NULL ||
     t2->oppositeVertex(this)->getEdge(t1->oppositeVertex(this)) != NULL)) return 0;

 Edge *e1 = t1->nextEdge(this);
 Edge *e3 = t2->nextEdge(this);
 v1->e0 = e3;
 v2->e0 = e1;
 v1 = t2->oppositeVertex(this);
 v2 = t1->oppositeVertex(this);
 t1->replaceEdge(e1, e3);
 t2->replaceEdge(e3, e1);
 t1->invert();
 t2->invert();
 e1->replaceTriangle(t1, t2);
 e3->replaceTriangle(t2, t1);

#ifdef USE_PER_TRIANGLE_COLORS
 Triangle *ot1 = t1->nextEdge(this)->oppositeTriangle(t1);
 Triangle *ot2 = t1->prevEdge(this)->oppositeTriangle(t1);
 Triangle *ot3 = t2->nextEdge(this)->oppositeTriangle(t2);
 Triangle *ot4 = t2->prevEdge(this)->oppositeTriangle(t2);
 uint32_t ct1 = t1->getColor();
 uint32_t ct2 = t2->getColor();
 uint32_t c1 = (ot1 != NULL) ? (ot1->getColor()) : (0);
 uint32_t c2 = (ot2 != NULL) ? (ot2->getColor()) : (0);
 uint32_t c3 = (ot3 != NULL) ? (ot3->getColor()) : (0);
 uint32_t c4 = (ot4 != NULL) ? (ot4->getColor()) : (0);
 UBYTE matches = 0, fmatches = 0;
 if (ct1 == c1) matches++;
 if (ct1 == c2) matches++;
 if (ct2 == c3) matches++;
 if (ct2 == c4) matches++;
 if (ct1 == c3) fmatches++;
 if (ct1 == c4) fmatches++;
 if (ct2 == c1) fmatches++;
 if (ct2 == c2) fmatches++;
 if (fmatches > matches) { t1->setColor(ct2); t2->setColor(ct1); }
#endif

 return 1;
}


////////////////////////// Edge collapse ////////////////////////

Vertex *Edge::collapseOnV1()
{
 Edge *e;
 Node *n;
 List *ve;
 Vertex *tv;

 Edge *e1 = (t1 != NULL)?(t1->nextEdge(this)):(NULL);
 Edge *e2 = (t1 != NULL)?(t1->prevEdge(this)):(NULL);
 Edge *e3 = (t2 != NULL)?(t2->nextEdge(this)):(NULL);
 Edge *e4 = (t2 != NULL)?(t2->prevEdge(this)):(NULL);
 Vertex *v3 = (e1 != NULL)?(e1->oppositeVertex(v2)):(NULL);
 Vertex *v4 = (e4 != NULL)?(e4->oppositeVertex(v2)):(NULL);
 Triangle *ta1 = (e1 != NULL)?(e1->oppositeTriangle(t1)):(NULL);
 Triangle *ta2 = (e2 != NULL)?(e2->oppositeTriangle(t1)):(NULL);
 Triangle *ta3 = (e3 != NULL)?(e3->oppositeTriangle(t2)):(NULL);
 Triangle *ta4 = (e4 != NULL)?(e4->oppositeTriangle(t2)):(NULL);

 if (v1->isOnBoundary() && v2->isOnBoundary())
  if (!(((ta1 || ta2) && !ta3 && !ta4) || ((ta3 || ta4) && !ta1 && !ta2))) return NULL;

 if (ta1 != NULL && ta2 != NULL && ta1->oppositeVertex(e1) == ta2->oppositeVertex(e2))
  return NULL;
 if (ta3 != NULL && ta4 != NULL && ta3->oppositeVertex(e3) == ta4->oppositeVertex(e4))
  return NULL;

 if (ta1 == NULL && ta2 == NULL) v1->e0 = e3;
 else v1->e0 = e2;

 if (v3 != NULL) v3->e0 = e2;
 if (v4 != NULL) v4->e0 = e3;

 ve = v2->VE();
 FOREACHVEEDGE(ve, e, n)
 {
  tv = e->oppositeVertex(v2);
  if (tv != v3 && tv != v4 && tv->getEdge(v1) != NULL) {delete(ve); return NULL;}
 }
 FOREACHVEEDGE(ve, e, n) if (e != this) e->replaceVertex(v2, v1);
 delete(ve);

 if (e2 != NULL) e2->replaceTriangle(t1, ta1);
 if (e3 != NULL) e3->replaceTriangle(t2, ta4);

 if (ta1 != NULL) ta1->replaceEdge(e1, e2);
 if (ta4 != NULL) ta4->replaceEdge(e4, e3);

 v2->e0 = NULL;						// v2 must be removed
 if (e4 != NULL) e4->v1 = e4->v2 = NULL;		// e4 must be removed
 if (e1 != NULL) e1->v1 = e1->v2 = NULL;		// e1 must be removed
 if (t1 != NULL) t1->e1 = t1->e2 = t1->e3 = NULL;	// t1 must be removed
 if (t2 != NULL) t2->e1 = t2->e2 = t2->e3 = NULL;	// t2 must be removed

 if (e2 != NULL && e2->t1 == NULL && e2->t2 == NULL)
 {
  v3->e0 = NULL;
  e2->v1 = e2->v2 = NULL;
 }
 if (e3 != NULL && e3->t1 == NULL && e3->t2 == NULL)
 {
  v4->e0 = NULL;
  e3->v1 = e3->v2 = NULL;
 }

 v4 = v1;							// This is the remaining vertex to be returned
 
 v2 = v1 = NULL;					// this edge must be removed

 return v4;
}

Vertex *Edge::collapseOnV2()
{
	invert();
	return collapseOnV1();
}

bool Edge::collapse(const Point& p)
{
 Vertex *r = collapseOnV1();
 if (r==NULL) return false;
 else r->setValue(&p);					// Average the collapse

 return true;
}


/*******************ANDREAS MODIFICATION*****************/
bool Edge::collapse(const Vertex* v)
{
  if(v == nullptr) return false;
  v == v1 ? v = collapseOnV1() : v = collapseOnV2();
  return v != nullptr;

}
/********************************************************/

bool Edge::collapse()
{
 return collapse(((*v1)+(*v2))/2);
}


///// Merge with another boundary edge /////

bool Edge::merge(Edge *e)
{
 if (t1 && t2) return 0;
 if (e->t1 && e->t2) return 0;
 Triangle *ot = (e->t1==NULL)?(e->t2):(e->t1);
 if (ot == getBoundaryTriangle()) return 0;
 if ((t1 && e->t1) || (t2 && e->t2)) e->invert();
 Vertex *ov1 = e->v1, *ov2 = e->v2;
 List *ve1=NULL, *ve2=NULL;
 Node *n;
 Edge *f, *f2;

 if (ov1 != v1)
 {
  ve1 = ov1->VE();
  FOREACHVEEDGE(ve1, f, n)
  {
   f2 = f->oppositeVertex(ov1)->getEdge(v1);
   if (f2 != NULL && (!f2->isOnBoundary() || !f->isOnBoundary()))
    {delete(ve1); return 0;}
  }
 }
 if (ov2 != v2)
 {
  ve2 = ov2->VE();
  FOREACHVEEDGE(ve2, f, n)
  {
   f2 = f->oppositeVertex(ov2)->getEdge(v2);
   if (f2 != NULL && (!f2->isOnBoundary() || !f->isOnBoundary()))
    {delete(ve1); delete(ve2); return 0;}
  }
 }

 if (ov1 != v1)
 {
  FOREACHVEEDGE(ve1, f, n) f->replaceVertex(ov1, v1);
  delete(ve1);
  ov1->e0 = NULL;
 }
 if (ov2 != v2)
 {
  FOREACHVEEDGE(ve2, f, n) f->replaceVertex(ov2, v2);
  delete(ve2);
  ov2->e0 = NULL;
 }
 ot->replaceEdge(e, this);
 ((t1==NULL)?(t1):(t2)) = ot;
 v1->e0 = v2->e0 = this;
 e->v1 = e->v2 = NULL;

 return 1;
}


///// Angle between the normal of incident triangles /////

double Edge::curvature() const
{
 if (!t1 || !t2) return -1.0;
 return t1->getDAngle(t2);
}


//// Dihedral angle

double Edge::dihedralAngle() const
{
 if (!t1 || !t2) return -1.0;
 Point nor1 = t1->getNormal();
 Point nor2 = t2->getNormal();
 if (nor1.isNull() || nor2.isNull()) return -1.0;
 double c = nor1.getAngle(&nor2);

 Vertex *ov = t2->oppositeVertex(this);
 if (((*ov)*nor1) - ((*v1)*nor1) < 0) return -(c - M_PI);

 return c+M_PI;
}

//// Min Angle among those of the two incident triangles ////

double Edge::delaunayMinAngle() const
{
 if (t1==NULL || t2==NULL) return 2*M_PI;
 if (squaredLength()==0) return 0;
 if (t1->nextEdge(this)->squaredLength() == 0) return 0;
 if (t1->prevEdge(this)->squaredLength() == 0) return 0;
 double a1 = t1->getAngle(v1);
 double a2 = t1->getAngle(v2);
 double a3 = t1->getAngle(t1->oppositeVertex(this));
 if (t2->nextEdge(this)->length()==0) return 0;
 if (t2->prevEdge(this)->length()==0) return 0;
 double a4 = t2->getAngle(v1);
 double a5 = t2->getAngle(v2);
 double a6 = t2->getAngle(t2->oppositeVertex(this));

 if (a1+a4 >= M_PI || a2+a5 >= M_PI) return 3*M_PI;
 return MIN(a1,(MIN(a2,(MIN(a3,(MIN(a4,(MIN(a5,a6)))))))));
}


// If edge is stitchable, merge it with its copy

bool Edge::stitch()
{
	// This function seems to be insufficient to stitch in every case !
	if (!isOnBoundary()) return 0;

	Triangle *t, *t0 = (t1 != NULL) ? (t1) : (t2);
	Vertex *v0;
	Edge *e1;

	for (v0 = v1; v0 != NULL; v0 = ((v0 == v1) ? (v2) : (NULL)))
	{
		e1 = this;
		t = t0;
		while (t != NULL)
		{
			e1 = t->nextEdge(e1); if (!e1->hasVertex(v0)) e1 = t->nextEdge(e1);
			t = e1->oppositeTriangle(t);
		}
		if (e1->oppositeVertex(v0) == oppositeVertex(v0))
		{
			t = (e1->t1 != NULL) ? (e1->t1) : (e1->t2);
			t->replaceEdge(e1, this);
			v1->e0 = v2->e0 = this;
			e1->v1 = e1->v2 = NULL;
			replaceTriangle(NULL, t);
			return 1;
		}
	}

	return 0;
}

bool Edge::overlaps() const
{
	if (t1 == NULL || t2 == NULL) return false;

	Vertex *ov = t2->oppositeVertex(this);
	if (ov->exactOrientation(t1->v1(), t1->v2(), t1->v3()) == 0 && ov->exactSameSideOnPlane(t1->oppositeVertex(this), v1, v2)) return true;
	else return false;
}

bool Edge::intersects(const Triangle *t) const
{
	if (t->hasEdge(this)) return false;
	
	Vertex *cv = (t->hasVertex(v1)) ? (v1) : ((t->hasVertex(v2)) ? (v2) : (NULL));
	if (cv) // If they share a vertex, intersection occurs if t's opposite edge intersect this edge
	{
		Edge *oe = t->oppositeEdge(cv);
		if (Point::pointInTriangle(oppositeVertex(cv), cv, oe->v1, oe->v2)) return true;
		else return (Point::segmentsIntersect(oe->v1, oe->v2, v1, v2));
	}
	else return Point::segmentIntersectsTriangle(v1, v2, t->v1(), t->v2(), t->v3());
}

coord Edge::getConvexity() const
{
	if (t1 == NULL || t2 == NULL) return DBL_MAX;
	else return (t1->oppositeVertex(this)->exactOrientation(t2->v3(), t2->v2(), t2->v1()));
}

//! Returns TRUE if edge has two incident triangles forming a flat and convex quadrilateral.
bool Edge::isFlatAndConvexQuadrilateral() const
{
	if (isOnBoundary()) return false; // Must be not on boundary
	Vertex *ov1 = t1->oppositeVertex(this);
	Vertex *ov2 = t2->oppositeVertex(this);
	if (v1->exactOrientation(v2, ov1, ov2) != 0) return false; // Must be flat
	if (!(!v1->exactSameSideOnPlane(v2, ov1, ov2) && ov1->exactMisalignment(v1, ov2) && ov1->exactMisalignment(v2, ov2))) return false; // Must be properly convex
	return true;
}

coord Edge::delaunayMinAngleSquaredSin() const
{
	if (!isFlatAndConvexQuadrilateral()) return DBL_MAX;

	Vertex *ov1 = t1->oppositeVertex(this);
	Vertex *ov2 = t2->oppositeVertex(this);

	Point l11 = (*ov1 - *v1), l12 = (*ov1 - *v2);
	coord b11 = l11.squaredLength();
	coord b12 = l12.squaredLength();
	coord min_sq_sin1 = ov1->squaredDistanceFromLine(v1,v2) / MAX(b11, b12); // Minimum angle at this edge on t1 (squared sin)

	Point l21 = (*ov2 - *v1), l22 = (*ov2 - *v2);
	coord b21 = l21.squaredLength();
	coord b22 = l22.squaredLength();
	coord min_sq_sin2 = ov2->squaredDistanceFromLine(v1, v2) / MAX(b21, b22); // Minimum angle at this edge on t2 (squared sin)

	coord sq_sin_ov1 = ((l11*l12)>0) ? ((l11&l12).squaredLength() / (b11*b12)) : (DBL_MAX); // Squared sin at ov1 (DBL_MAX if obtuse)
	coord sq_sin_ov2 = ((l21*l22)>0) ? ((l21&l22).squaredLength() / (b21*b22)) : (DBL_MAX); // Squared sin at ov2 (DBL_MAX if obtuse)

	coord min_sq_ov = MIN(sq_sin_ov1, sq_sin_ov2);
	coord min_sq_t = MIN(min_sq_sin1, min_sq_sin2);

	return MIN(min_sq_ov, min_sq_t);
}



//////// Length-based edge comparison for qsort //////////

int edgeCompare(const void *a, const void *b)
{
	coord la = ((Edge *)a)->squaredLength();
	coord lb = ((Edge *)b)->squaredLength();

	if (la<lb) return -1;
	if (la>lb) return 1;

	return 0;
}


//////// Lexycographic edge comparison for qsort //////////

int lexEdgeCompare(const void *a, const void *b)
{
	Vertex *va1 = ((Edge *)a)->v1;
	Vertex *va2 = ((Edge *)a)->v2;
	Vertex *vb1 = ((Edge *)b)->v1;
	Vertex *vb2 = ((Edge *)b)->v2;

	if (xyzCompare(va1, va2) > 0) p_swap((void **)&va1, (void **)&va2);
	if (xyzCompare(vb1, vb2) > 0) p_swap((void **)&vb1, (void **)&vb2);

	int ca = xyzCompare(va1, vb1);

	if (ca == 0) return xyzCompare(va2, vb2);

	return ca;
}


//////// Vertex-based edge comparison for qsort //////////

int vtxEdgeCompare(const void *a, const void *b)
{
	Vertex *va1 = ((Edge *)a)->v1;
	Vertex *va2 = ((Edge *)a)->v2;
	Vertex *vb1 = ((Edge *)b)->v1;
	Vertex *vb2 = ((Edge *)b)->v2;
	Vertex *tmp;

	if (va2<va1) { tmp = va1; va1 = va2; va2 = tmp; }
	if (vb2<vb1) { tmp = vb1; vb1 = vb2; vb2 = tmp; }
	if (va1<vb1) return -1;
	if (va1>vb1) return 1;
	if (va2<vb2) return -1;
	if (va2>vb2) return 1;
	return 0;
}

} //namespace T_MESH
