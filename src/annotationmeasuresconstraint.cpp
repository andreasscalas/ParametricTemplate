#include "annotationmeasuresconstraint.h"

#include <drawableboundingmeasure.h>
#include <utilities.h>
#include <surfaceannotation.h>
#include <drawableheightmeasure.h>
AnnotationMeasuresConstraint::AnnotationMeasuresConstraint()
{

}

void AnnotationMeasuresConstraint::constrain()
{


}

void AnnotationMeasuresConstraint::print(std::ostream &os)
{
    AnnotationsRelationship::print(os);

}

void AnnotationMeasuresConstraint::checkConstraint()
{
    dynamic_cast<DrawableAttribute*>(attribute1)->update();
    dynamic_cast<DrawableAttribute*>(attribute2)->update();
    double measure1 = *static_cast<double*>(attribute1->getValue());
    double measure2 = *static_cast<double*>(attribute2->getValue());
    double proportion = measure1 / measure2;
    if(proportion < minValue || proportion > maxValue)
        dynamic_cast<DrawableAttribute*>(attribute1)->setError(true);
    else
        dynamic_cast<DrawableAttribute*>(attribute1)->setError(false);

}
