#include "annotationsconstraint.h"
#include <utilities.h>
#include <pointannotation.h>
#include <lineannotation.h>
#include <surfaceannotation.h>

AnnotationsConstraint::AnnotationsConstraint()
{

}

AnnotationsConstraint::AnnotationsConstraint(AnnotationsRelationship *p) : AnnotationsRelationship(*p)
{

}

std::vector<Annotation *> AnnotationsConstraint::getAnnotations() const
{
    return annotations;
}

void AnnotationsConstraint::setAnnotations(const std::vector<Annotation *> &value)
{
    annotations = value;
}

void AnnotationsConstraint::constrain()
{

}

void AnnotationsConstraint::checkConstraint()
{
}

void AnnotationsConstraint::print(std::ostream &os)
{
    AnnotationsRelationship::print(os);

}

ConstraintSolver *AnnotationsConstraint::getSolver() const
{
    return solver;
}

void AnnotationsConstraint::setSolver(ConstraintSolver *value)
{
    solver = value;
}

std::vector<int> AnnotationsConstraint::getInvolvedVerticesIDs(Annotation *annotation)
{
    std::vector<IMATI_STL::Vertex*> involvedVertices = annotation->getInvolvedVertices();
    std::vector<int> involvedIDs;
    for (unsigned int i = 0; i < involvedVertices.size(); i++)
        involvedIDs.push_back( static_cast<int>(annotation->getMesh()->getPointId(involvedVertices[i])));

    return involvedIDs;
}
