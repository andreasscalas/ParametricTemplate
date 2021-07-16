#ifndef ANNOTATIONSCONSTRAINT_H
#define ANNOTATIONSCONSTRAINT_H

#include <vector>

#include <annotation.h>
#include <constraintsolver.h>
#include <annotationsrelationship.h>


class ConstraintSolver;
class AnnotationsConstraint : virtual public AnnotationsRelationship
{
public:
    AnnotationsConstraint();
    AnnotationsConstraint(AnnotationsRelationship*);

    virtual void constrain();
    virtual void checkConstraint();
    virtual void print(std::ostream &os);

    std::vector<Annotation *> getAnnotations() const;
    void setAnnotations(const std::vector<Annotation *> &value);
    double getError();


    ConstraintSolver *getSolver() const;
    void setSolver(ConstraintSolver *value);

protected:
    ConstraintSolver* solver;

    void constrainSelf();
    void constrainSet();
    std::vector<int> getInvolvedVerticesIDs(Annotation* annotation);
};

#endif // ANNOTATIONSCONSTRAINT_H
