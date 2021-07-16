#ifndef VERTICESSELECTIONSTYLE_H
#define VERTICESSELECTIONSTYLE_H

#include <drawablemesh.h>
#include <pointannotation.h>

#include <vector>
#include <map>
#include <string>

#include <vtkSmartPointer.h>
#include <vtkPointPicker.h>
#include <vtkInteractorStyleRubberBandPick.h>
#include <vtkPolyData.h>
#include <QVTKWidget.h>

#define VTKISRBP_ORIENT 0
#define VTKISRBP_SELECT 1

/**
 * @brief The VerticesSelectionStyle class controls the interaction with the points of a mesh
 */
class VerticesSelectionStyle : public vtkInteractorStyleRubberBandPick {

private:
    constexpr static double RADIUS_RATIO = 100;

    vtkSmartPointer<vtkPropAssembly> assembly;          //Assembly of actors
    vtkSmartPointer<vtkPropAssembly> sphereAssembly;    //Assembly of spheres
    vtkSmartPointer<vtkPolyData> Points;
	std::map<unsigned long, bool>* pointsSelectionStatus;
	std::vector<unsigned long> selectedPoints;
    vtkSmartPointer<vtkPointPicker> pointPicker;        //The point picker
	QVTKWidget* qvtkwidget;
	bool selectionMode;
	bool visiblePointsOnly;
    bool leftPressed;
    double sphereRadius;
    DrawableMesh* mesh;
    PointAnnotation* annotation;

public:
    static VerticesSelectionStyle* New();

    VerticesSelectionStyle();

    vtkTypeMacro(VerticesSelectionStyle, vtkInteractorStyleRubberBandPick)

    virtual void OnRightButtonDown() override;

	virtual void OnLeftButtonDown() override;

	virtual void OnLeftButtonUp() override;

    virtual void OnMouseMove() override;

	void resetSelection();

	void defineSelection(std::vector<unsigned long> selected);

    void modifySelectedPoints();

    void finalizeAnnotation(unsigned int id, std::string tag, unsigned char color[]);

    vtkSmartPointer<vtkPolyData> getPoints() const;
    void setPoints(const vtkSmartPointer<vtkPolyData>& value);
	std::map<unsigned long, bool>* getSelectedPoints() const;
	void setSelectedPoints(std::map<unsigned long, bool>* value);
	vtkSmartPointer<vtkPointPicker> getPointPicker() const;
	void setPointPicker(const vtkSmartPointer<vtkPointPicker>& value);
	bool getSelectionMode() const;
	void setSelectionMode(bool value);
	bool getVisiblePointsOnly() const;
	void setVisiblePointsOnly(bool value);
    DrawableMesh* getMesh() const;
    void setMesh(DrawableMesh* value);
    vtkSmartPointer<vtkPropAssembly> getAssembly() const;
    void setAssembly(const vtkSmartPointer<vtkPropAssembly>& value);
	QVTKWidget* getQvtkwidget() const;
    void setQvtkwidget(QVTKWidget* value);
    std::map<unsigned long, bool> *getPointsSelectionStatus() const;
    void setPointsSelectionStatus(std::map<unsigned long, bool> *value);

};



#endif // VERTICESSELECTIONSTYLE_H
