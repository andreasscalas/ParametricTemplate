#include "attributeconstraint.h"
//#include <semanticattribute.h>
#include <geometricattribute.h>

AttributeConstraint::AttributeConstraint()
{

}

void AttributeConstraint::constrain()
{

}

double AttributeConstraint::getError()
{

}

std::vector<Attribute *> AttributeConstraint::getAttributes() const
{
    return attributes;
}

void AttributeConstraint::setAttributes(const std::vector<Attribute *> &value)
{
    attributes = value;
}

SemanticRelationshipType AttributeConstraint::getType() const
{
    return type;
}

void AttributeConstraint::setType(const SemanticRelationshipType &value)
{
    type = value;
}

