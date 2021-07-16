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
    virtual double getError() = 0;
    double getWeight() const { return weight; }
    void setWeight(double value){ weight = value; }
    double getMinValue() const{ return minValue; }
    void setMinValue(double value){ minValue = value; }
    double getMaxValue() const{ return maxValue; }
    void setMaxValue(double value){ maxValue = value; }
    ConstraintSolver *getSolver() const{ return solver;}
    void setSolver(ConstraintSolver *value){ solver = value; }

protected:
    double weight;
    double minValue, maxValue;
    ConstraintSolver* solver;
};

#endif // SEMANTICCONSTRAINT_H
