#ifndef SEMANTICCONSTRAINT_H
#define SEMANTICCONSTRAINT_H

#include <vector>
#include <constraintsolver.h>

class ConstraintSolver;
class SemanticConstraint
{
public:
    SemanticConstraint() {}
    virtual ~SemanticConstraint() {}
    virtual void constrain() = 0;
};

#endif // SEMANTICCONSTRAINT_H
