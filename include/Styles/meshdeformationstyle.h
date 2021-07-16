#ifndef MESHDEFORMATIONSTYLE_H
#define MESHDEFORMATIONSTYLE_H

#include <drawablemesh.h>
#include <barycentriccoordinates.h>
#include <constraintsolver.h>
#include <annotationsconstraint.h>
#include <map>

#include <vtkSmartPointer.h>
#include <vtkPointPicker.h>
#include <vtkActor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPolyData.h>
#include <QVTKWidget.h>

#define VTK_CREATE(type, name) \
    vtkSmartPointer<type> name = vtkSmartPointer<type>::New()

/**
 * @brief The MeshDeformationStyle class controls the interaction with the points of a cage
 */
class MeshDeformationStyle : public vtkInteractorStyleTrackballCamera{

public:
        const double EPSILON = 1e-5;
        vtkSmartPointer<vtkPolyData> data;                  //Linking between coordinates and ids
        vtkSmartPointer<vtkPointPicker> pointPicker;        //The point picker
        vtkSmartPointer<vtkPropAssembly> assembly;              //Assembly of actors
        vtkSmartPointer<vtkActor> outlineActor;
        QVTKWidget* qvtkwidget;
        DrawableMesh* model;
        DrawableMesh* cage;                                 //The cage
        ConstraintSolver* solver;
        BarycentricCoordinates* coords;
        std::vector<AnnotationsConstraint*> semanticConstraints;
        bool coordsComputed;
        bool constrained;
        bool stretch;
        bool stretchBegun;
        bool shrinkBegun;
        int i = 0, minPos = -1, maxPos = -1;

        bool translation, rotation;                                //True if some point has been picked
        std::map<ulong, bool>* pointsSelectionStatus;              //Id of the picked point
        double clickPos[3];

        static MeshDeformationStyle* New();

        MeshDeformationStyle();

        vtkTypeMacro(MeshDeformationStyle, vtkInteractorStyleTrackballCamera)

        void drawBB(vtkSmartPointer<vtkPropAssembly> canvas);

        void OnMouseWheelForward() override;
        void OnMouseWheelBackward() override;

        void OnMouseMove() override;

        void OnRightButtonUp() override;

        void OnRightButtonDown() override;

        void OnLeftButtonUp() override;

        void OnLeftButtonDown() override;

        QVTKWidget *getQvtkwidget() const;

        void setQvtkwidget(QVTKWidget *value);

        vtkSmartPointer<vtkPropAssembly> getAssembly() const;

        void setAssembly(const vtkSmartPointer<vtkPropAssembly> &value);

        DrawableMesh *getModel() const;

        void setModel(DrawableMesh *value);

        DrawableMesh *getCage() const;

        void setCage(DrawableMesh *value);

        std::map<ulong, bool> *getSelectedPoints() const;

        void setSelectedPoints(std::map<ulong, bool> *value);

        vtkSmartPointer<vtkPolyData> getData() const;

        void setData(const vtkSmartPointer<vtkPolyData> &value);

        bool getCoordsComputed() const;

        void setCoordsComputed(bool value);

        void checkConstraintsSeamlessly();
        std::vector<unsigned int> getClosenessConstraintsID() const;
        void setClosenessConstraintsID(const std::vector<unsigned int> &value);
        ConstraintSolver *getSolver() const;
        void setSolver(ConstraintSolver *value);
        bool getConstrained() const;
        void setConstrained(bool value);

        bool getStretch() const;
        void setStretch(bool value);

        std::vector<AnnotationsConstraint *> getSemanticConstraints() const;
        void setSemanticConstraints(const std::vector<AnnotationsConstraint *> &value);

        BarycentricCoordinates *getCoords() const;
        void setCoords(BarycentricCoordinates *value);

protected:
        void applyDeformation();
};
#endif // MESHDEFORMATIONSTYLE_H
