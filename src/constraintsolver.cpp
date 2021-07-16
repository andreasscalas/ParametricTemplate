#include "constraintsolver.h"
#include <chrono>

ConstraintSolver::ConstraintSolver()
{
    reachedConstraintID = 0;
}

ConstraintSolver::~ConstraintSolver()
{
    this->clear();
}

void ConstraintSolver::clear()
{
}
void ConstraintSolver::checkConstraints()
{
    for(unsigned int i = 0; i < semanticConstraints.size(); i++)
    {
        AnnotationsConstraint* c = semanticConstraints[i];
        c->checkConstraint();
    }
}


int ConstraintSolver::addSemanticConstraint(AnnotationsConstraint *c)
{
    semanticConstraints.push_back(c);
    c->setId(reachedConstraintID++);
    return static_cast<int>(semanticConstraints.size() - 1);
}

bool ConstraintSolver::removeSemanticConstraint(const unsigned int cid)
{
    semanticConstraints.erase(semanticConstraints.begin() + cid);
}

bool ConstraintSolver::setSemanticConstraint(const unsigned int cid, AnnotationsConstraint *c)
{
    if(semanticConstraints.size() > cid){
        semanticConstraints[cid] = c;
        return true;
    }
    return false;
}

AnnotationsConstraint *ConstraintSolver::getSemanticConstraint(const unsigned int cid)
{
    return semanticConstraints[cid];
}
