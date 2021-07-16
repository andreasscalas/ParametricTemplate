#ifndef CONSTRAINTSOLVER_H
#define CONSTRAINTSOLVER_H

#include <vector>
#include <memory>
#include <annotationsconstraint.h>

class AnnotationsConstraint;
class ConstraintSolver
{
public:

    ConstraintSolver();
    ~ConstraintSolver();

    void clear();

    //Temporary function
    void checkConstraints();

    bool initialize();

    bool solve(unsigned int iterationNumber);

    int addSemanticConstraint(AnnotationsConstraint* c);

    bool removeSemanticConstraint(const unsigned int cid);

    bool setSemanticConstraint(const unsigned int cid, AnnotationsConstraint*);

    AnnotationsConstraint* getSemanticConstraint(const unsigned int cid);

private:
    unsigned int reachedConstraintID;
    std::vector<AnnotationsConstraint*> semanticConstraints;


};

#endif // CONSTRAINTSOLVER_H
