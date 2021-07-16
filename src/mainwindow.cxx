#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <time.h>
#include <mutex>
#include <fstream>
#include <istream>

#include <annotationwindow.h>
#include <annotationmeasuresconstraint.h>
#include <annotationdialog.h>
#include <correspondencesdialog.h>
#include <utilities.h>
#include <meanvaluecoordinatesju.h>
//#include <greencoordinates.h>
//#include <drawableboundingmeasure.h>

#include <vtkOBBTree.h>
#include <vtkTetra.h>
#include <vtkUnstructuredGrid.h>
#include <vtkCamera.h>
#include <vtkIdFilter.h>
#include <vtkPlane.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkOutlineSource.h>
#include <vtkAreaPicker.h>
#include <vtkRegularPolygonSource.h>
#include <vtkInteractorStyleImage.h>
#include <vtkProperty.h>
#include <vtkLine.h>
#include <vtkCutter.h>
//#include <vtkVectorText.h>
using namespace IMATI_STL;
using namespace std;

vtkStandardNewMacro(MeshDeformationStyle)
vtkStandardNewMacro(CageVerticesSelectionStyle)

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow){

    ui->setupUi(this);
    this->init();
    this->drawInitialTetrahedron();
    ad = new AnnotationDialog(this);
    cd = new AnnotationConstraintDialog(this);
    this->ui->line->setVisible(false);
    this->ui->genericPalette->setVisible(true);
    this->lui->setVisible(true);
    this->ui->actionLayerDialog->setChecked(true);
    iren = static_cast<QVTKInteractor*>(this->ui->qvtkWidget->GetRenderWindow()->GetInteractor());
    //Signals-Slots connection
    connect(this->ui->actionOpenMesh, SIGNAL(triggered()), this, SLOT(slotOpenMesh()));
    connect(this->ui->actionOpenCage, SIGNAL(triggered()), this, SLOT(slotOpenCage()));
    connect(this->ui->actionOpenBC, SIGNAL(triggered()), this, SLOT(slotOpenBC()));
    connect(this->ui->actionOpenConstraints, SIGNAL(triggered()), this, SLOT(slotOpenConstraints()));
    connect(this->ui->actionOpenFragment, SIGNAL(triggered()), this, SLOT(slotOpenFragment()));
    connect(this->ui->actionSaveMesh, SIGNAL(triggered()), this, SLOT(slotSaveMesh()));
    connect(this->ui->actionSaveCage, SIGNAL(triggered()), this, SLOT(slotSaveCage()));
    connect(this->ui->actionSaveBC, SIGNAL(triggered()), this, SLOT(slotSaveBC()));
    connect(this->ui->actionSaveConstraints, SIGNAL(triggered()), this, SLOT(slotSaveConstraints()));
    connect(this->ui->actionClearAll, SIGNAL(triggered()), this, SLOT(slotClearAll()));
    connect(this->ui->actionClearMesh, SIGNAL(triggered()), this, SLOT(slotClearMesh()));
    connect(this->ui->actionClearCage, SIGNAL(triggered()), this, SLOT(slotClearCage()));
    connect(this->ui->actionClearCamera, SIGNAL(triggered()), this, SLOT(slotClearCamera()));
    connect(this->ui->actionClose, SIGNAL(triggered()), this, SLOT(slotClose()));
    connect(this->ui->actionGenerate, SIGNAL(triggered()), this, SLOT(slotGenerate()));
    connect(this->ui->actionComputeCoords, SIGNAL(triggered()), this, SLOT(slotComputeCoords()));
    connect(this->ui->actionShowMesh, SIGNAL(triggered(bool)), this, SLOT(slotShowMesh(bool)));
    connect(this->ui->actionShowMeshSurface, SIGNAL(triggered(bool)), this, SLOT(slotShowMeshSurface(bool)));
    connect(this->ui->actionShowMeshWireframe, SIGNAL(triggered(bool)), this, SLOT(slotShowMeshWireframe(bool)));
    connect(this->ui->actionShowMeshPoints, SIGNAL(triggered(bool)), this, SLOT(slotShowMeshPoints(bool)));
    connect(this->ui->actionShowMeshAnnotations, SIGNAL(triggered(bool)), this, SLOT(slotShowMeshAnnotations(bool)));
    connect(this->ui->actionShowCage, SIGNAL(triggered(bool)), this, SLOT(slotShowCage(bool)));
    connect(this->ui->actionShowCageSurface, SIGNAL(triggered(bool)), this, SLOT(slotShowCageSurface(bool)));
    connect(this->ui->actionShowCageWireframe, SIGNAL(triggered(bool)), this, SLOT(slotShowCageWireframe(bool)));
    connect(this->ui->actionShowCagePoints, SIGNAL(triggered(bool)), this, SLOT(slotShowCagePoints(bool)));
    connect(this->ui->actionMeshColor, SIGNAL(triggered()), this, SLOT(slotMeshColor()));
    connect(this->ui->actionCageColor, SIGNAL(triggered()), this, SLOT(slotCageColor()));
    connect(this->ui->actionSlicerPalette, SIGNAL(triggered(bool)), this, SLOT(slotSlicerPalette(bool)));
    connect(this->ui->actionLayerDialog, SIGNAL(triggered(bool)), this, SLOT(slotLayerDialog(bool)));
    connect(this->ui->actionCamera, SIGNAL(triggered(bool)), this, SLOT(slotCamera(bool)));
    connect(this->ui->actionVerticesSelection, SIGNAL(triggered(bool)), this, SLOT(slotVerticesSelection(bool)));
    connect(this->ui->actionVerticesDeselection, SIGNAL(triggered(bool)), this, SLOT(slotVerticesDeselection(bool)));
    connect(this->ui->actionMeshDeformation, SIGNAL(triggered(bool)), this, SLOT(slotMeshDeformation(bool)));
    connect(this->ui->actionStretch, SIGNAL(triggered(bool)), this, SLOT(slotStretch(bool)));
    connect(this->ui->actionGreen, SIGNAL(triggered()), this, SLOT(slotGreen()));
    connect(this->ui->actionMean, SIGNAL(triggered()), this, SLOT(slotMeanValue()));
    connect(this->ui->actionVisible, SIGNAL(triggered(bool)), this, SLOT(slotVisible(bool)));
    connect(this->ui->actionSlicer, SIGNAL(triggered()), this, SLOT(slotChangeToSlicer()));
    connect(this->ui->actionCheckConstraints, SIGNAL(triggered()), this, SLOT(slotCheckConstraints()));
    connect(this->lui, SIGNAL(updateMeshView(DrawableMesh*)), this, SLOT(slotUpdateMeshView(DrawableMesh*)));
    connect(this->lui, SIGNAL(deleteMesh(DrawableMesh*)), this, SLOT(slotDeleteMesh(DrawableMesh*)));
    connect(this->lui, SIGNAL(editAnnotations(DrawableMesh*)), this, SLOT(slotEditAnnotations(DrawableMesh*)));
    connect(this->lui, SIGNAL(fitRigidly(DrawableMesh*)), this, SLOT(slotFitRigidly(DrawableMesh*)));
    connect(this->lui, SIGNAL(adaptTemplate(DrawableMesh*)), this, SLOT(slotAdaptTemplate(DrawableMesh*)));
    connect(this->cd, SIGNAL(addSemanticRelationship(std::string, double, double, double, unsigned int, unsigned int, bool)), this, SLOT(slotAddAnnotationsConstraint(std::string, double, double, double, unsigned int, unsigned int, bool)));

}

MainWindow::~MainWindow(){
    delete ui;
    delete ad;
    delete cd;
    delete model;
    delete cage;
    delete solver;
    pointsSelectionStatus.clear();
    boundingBox.clear();
}

void MainWindow::init(){

    this->currentPath = "./";
    this->newWindow = true;
    this->cageLoaded = false;
    this->showBoundingBox = false;
    this->showSkeleton = false;
    this->reachedContraintId = 0;
    this->constrained = false;
    this->coordsComputed = false;
    this->firstFragmentPositioned = false;
    this->fitTemplateOnFragments = true;
    this->lui = new LayerDialog(this->ui->genericPalette);
    this->lui->setVisible(false);
    this->lui->setConstraintsImposed(false);
    this->levelOfDetail = MAX_LOD;
    this->allowedError = 0;
    this->coords = nullptr;
    this->coordType = MVC;
    this->model = nullptr;
    this->meshLoaded = false;
    this->cage = nullptr;
    this->cageLoaded = false;
    this->cagePoints = vtkSmartPointer<vtkPolyData>::New();
    this->modelPoints = vtkSmartPointer<vtkPolyData>::New();
    this->modelTriangles = vtkSmartPointer<vtkPolyData>::New();
    this->meshAssembly = vtkSmartPointer<vtkPropAssembly>::New();
    this->visualPropertiesAssembly = vtkSmartPointer<vtkPropAssembly>::New();
    this->measuresAssembly = vtkSmartPointer<vtkPropAssembly>::New();
    this->selectionStyle = vtkSmartPointer<CageVerticesSelectionStyle>::New();
    this->deformationStyle = vtkSmartPointer<MeshDeformationStyle>::New();
    this->solver = nullptr;
    this->pointsSelectionStatus.clear();
    this->ren = vtkSmartPointer<vtkRenderer>::New();
    this->ren->SetRenderWindow(this->ui->qvtkWidget->GetRenderWindow());
    this->ren->GetRenderWindow()->Render();
    this->ren->AddActor(meshAssembly);
    this->ren->SetBackground(1.0, 1.0, 1.0);
    this->ren->SetLayer(0);
    this->ui->qvtkWidget->GetRenderWindow()->AddRenderer(ren);
    this->ui->actionSlicerPalette->setChecked(false);
    this->ui->actionInfoPalette->setChecked(false);
    this->ui->actionVisible->setChecked(true);
    this->ui->actionCamera->setChecked(true);
    this->ui->actionVerticesSelection->setChecked(false);
    this->ui->actionVerticesDeselection->setChecked(false);
    this->ui->actionMeshDeformation->setChecked(false);
    this->ui->actionStretch->setChecked(false);
    this->ui->actionGreen->setChecked(false);
    this->ui->actionMean->setChecked(true);
    this->ui->actionDelete->setChecked(false);
    this->ui->actionRectSelection->setChecked(false);
    this->ui->actionLassoSelection->setChecked(false);
    this->ui->actionAnnotation->setChecked(false);
    this->ui->actionCamera->setEnabled(false);
    this->ui->actionVerticesSelection->setEnabled(false);
    this->ui->actionVerticesDeselection->setEnabled(false);
    this->ui->actionMeshDeformation->setEnabled(false);
    this->ui->actionStretch->setEnabled(false);
    this->ui->actionGreen->setEnabled(false);
    this->ui->actionMean->setEnabled(false);
    this->ui->actionDelete->setEnabled(false);
    this->ui->actionLineSelection->setEnabled(false);
    this->ui->actionRectSelection->setEnabled(false);
    this->ui->actionLassoSelection->setEnabled(false);
    this->ui->actionAnnotation->setEnabled(false);
    this->ui->actionShowMesh->setEnabled(false);
    this->ui->actionShowCage->setEnabled(false);
    this->ui->actionShowMesh->setChecked(false);
    this->ui->actionShowCage->setChecked(false);
    this->ui->actionSlicerPalette->setEnabled(false);
    this->ui->actionSlicerPalette->setChecked(false);
    this->ui->actionInfoPalette->setEnabled(false);
    this->ui->actionInfoPalette->setChecked(false);
    this->ui->verticesLabel->setText("Vertices: ");
    this->ui->edgesLabel->setText("Edges: ");
    this->ui->trianglesLabel->setText("Triangles: ");
    this->ui->cageVerticesLabel->setText("Cage vertices: ");
    this->ui->cageEdgesLabel->setText("Cage edges: ");
    this->ui->cageTrianglesLabel->setText("Cage triangles: ");
    this->initialCamera = vtkSmartPointer<vtkCamera>::New();
    this->initialCamera->DeepCopy(ren->GetActiveCamera());

}

void MainWindow::write(std::string message){

//    vtkSmartPointer<vtkVectorText> text = vtkSmartPointer<vtkVectorText>::New();
//    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
//    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
//    text->SetText(message.c_str());
//    mapper->SetInputConnection(text->GetOutputPort());
//    actor->SetMapper(mapper);
//    actor->GetProperty()->SetColor(0,0,0);
//    this->meshAssembly->AddPart(actor);
//    this->ren->ResetCamera();
//    this->ui->qvtkWidget->update();

}

void MainWindow::drawInitialTetrahedron()
{
    vtkSmartPointer< vtkPoints > points = vtkSmartPointer< vtkPoints > :: New();
    points->InsertNextPoint(-0.32, -0.75, 0);
    points->InsertNextPoint(1, 0, -0.5);
    points->InsertNextPoint(-0.32, 0.75, 0);
    points->InsertNextPoint(-0.32, 0, -1.52);

    vtkSmartPointer<vtkUnstructuredGrid> unstructuredGrid1 = vtkSmartPointer<vtkUnstructuredGrid>::New();
     unstructuredGrid1->SetPoints(points);

     vtkIdType ptIds[] = {0, 1, 2, 3};
     unstructuredGrid1->InsertNextCell( VTK_TETRA, 4, ptIds );

    vtkSmartPointer<vtkTetra> tetra = vtkSmartPointer<vtkTetra>::New();
    tetra->GetPointIds()->SetId(0, 0);
    tetra->GetPointIds()->SetId(1, 1);
    tetra->GetPointIds()->SetId(2, 2);
    tetra->GetPointIds()->SetId(3, 3);
    vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();
    cellArray->InsertNextCell(tetra);
    vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New();
    mapper->SetInputData(unstructuredGrid1);

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(0.752941176,0.752941176,0.752941176);
    this->meshAssembly->AddPart(actor);
    this->ren->ResetCamera();
    this->ui->qvtkWidget->update();
}

void MainWindow::updateView(){

    vtkSmartPointer<vtkPlane> intersectingPlane = vtkSmartPointer<vtkPlane>::New();
    vtkSmartPointer<vtkCutter> cutter = vtkSmartPointer<vtkCutter>::New();
    vtkSmartPointer<vtkPolyData> poly = vtkSmartPointer<vtkPolyData>::New();
    intersectingPlane->SetOrigin(center.x, center.y, center.z);
    intersectingPlane->SetNormal(normal.x, normal.y, normal.z);
    this->ren->Clear();

    if(cage != nullptr)
        this->cage->draw(meshAssembly);
    this->meshAssembly->RemovePart(visualPropertiesAssembly);
    this->visualPropertiesAssembly = vtkSmartPointer<vtkPropAssembly>::New();

    for(unsigned int i = 0; i < fragments.size(); i++)
        for(unsigned int j = 0; j < fragments[i]->getAnnotations().size(); j++)
            for(unsigned int k = 0; k < fragments[i]->getAnnotations()[j]->getAttributes().size(); k++)
                dynamic_cast<DrawableAttribute*>(fragments[i]->getAnnotations()[j]->getAttributes()[k])->setRenderer(ren);
    if(this->showBoundingBox){

        if(this->boundingBox.size() == 0){
            vector<double> bb = Utilities::getOBB(model);
            this->boundingBox.insert(this->boundingBox.end(), bb.begin(), bb.end());
        }

        vtkSmartPointer<vtkPolyData> boundingBoxData = vtkSmartPointer<vtkPolyData>::New();
        vtkSmartPointer<vtkPolyDataMapper> boundingBoxMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        vtkSmartPointer<vtkActor> boundingBoxActor = vtkSmartPointer<vtkActor>::New();
        vtkSmartPointer<vtkOutlineSource> boundingBoxSource = vtkSmartPointer<vtkOutlineSource>::New();

        boundingBoxSource->SetBoxTypeToOriented();
        boundingBoxSource->SetCorners(this->boundingBox.data());
        boundingBoxSource->Update();
        boundingBoxMapper->SetInputConnection(boundingBoxSource->GetOutputPort());
        boundingBoxActor->SetMapper(boundingBoxMapper);
        boundingBoxActor->GetProperty()->SetColor(1, 0, 0);
        this->visualPropertiesAssembly->AddPart(boundingBoxActor);
    }


    this->ui->qvtkWidget->update();

}

void MainWindow::clear(){

    this->setWindowTitle("MainWindow");
    this->meshAssembly = vtkSmartPointer<vtkPropAssembly>::New();
    this->newWindow = true;
    this->cageLoaded = false;
    this->firstFragmentPositioned = false;
    if(model != nullptr)
        delete model;
    this->model = nullptr;
    this->model = new DrawableMesh();
    this->meshLoaded = false;
    this->lui->removeCage(cage);
    if(cage != nullptr)
        delete cage;
    if(this->cage != nullptr)
        delete this->cage;
    this->cage = nullptr;
    this->cagePoints = vtkSmartPointer<vtkPolyData>::New();
    this->modelPoints = vtkSmartPointer<vtkPolyData>::New();
    this->pointsSelectionStatus.clear();
    if(solver != nullptr)
        delete solver;
    this->modelTriangles = vtkSmartPointer<vtkPolyData>::New();
    this->selectionStyle = vtkSmartPointer<CageVerticesSelectionStyle>::New();
    meshAssembly = vtkSmartPointer<vtkPropAssembly>::New();
    visualPropertiesAssembly = vtkSmartPointer<vtkPropAssembly>::New();
    this->resetCamera();
    this->ui->actionSlicerPalette->setChecked(false);
    this->ui->actionInfoPalette->setChecked(false);
    this->ui->actionVerticesSelection->setEnabled(false);
    this->ui->actionVerticesDeselection->setEnabled(false);
    this->ui->actionMeshDeformation->setEnabled(false);
    this->ui->actionStretch->setEnabled(false);
    this->ui->actionAddVerticesRectangle->setEnabled(false);
    this->ui->verticesLabel->setText("Vertices: ");
    this->ui->edgesLabel->setText("Edges: ");
    this->ui->trianglesLabel->setText("Triangles: ");
    this->ui->cageVerticesLabel->setText("Cage vertices: ");
    this->ui->cageEdgesLabel->setText("Cage edges: ");
    this->ui->cageTrianglesLabel->setText("Cage triangles: ");
    vtkSmartPointer<vtkInteractorStyleImage> imageStyle = vtkSmartPointer<vtkInteractorStyleImage>::New();
    //this->clearAll();

}

void MainWindow::slotOpenFile(){

    QString filename = QFileDialog::getOpenFileName(nullptr,
                           "Choose a 3D model",
                           QString::fromStdString(currentPath),
                           "STL(*.stl);;PLY(*.ply);;OBJ(*.obj);; VRML(.wrl);; IV(.iv);; OFF(.off);; EFF(.eff);; VERTRI(.tri)");

    if (!filename.isEmpty()){

        std::cout << "loading: " << filename.toStdString() << std::endl;
        this->clearAll();

        QFileInfo info(filename);
        currentPath = info.absolutePath().toStdString();

        if(model->load(filename.toStdString().c_str()) != 0){
            this->write("Can't open mesh file");
        }else{
            this->model->draw(meshAssembly);
            this->meshLoadedPreparation();
        }
    }

}

void MainWindow::slotOpenMesh(){

    QString filename = QFileDialog::getOpenFileName(nullptr,
                           "Choose a 3D model",
                           QString::fromStdString(currentPath),
                           "PLY(*.ply);;STL(*.stl);;OBJ(*.obj);; VRML(.wrl);; IV(.iv);; OFF(.off);; EFF(.eff);; VERTRI(.tri);; ALL(*)");

    if (!filename.isEmpty()){

        QFileInfo info(filename);
        modelFilename = info.fileName();
        currentPath = info.absolutePath().toStdString();
        std::cout << "loading: " << filename.toStdString() << std::endl;
        this->clearAll();
        this->model = new DrawableMesh();

        if(this->model->load(filename.toStdString().c_str()) != 0)
            this->write("Can't open mesh file");
        else
            this->meshLoadedPreparation();

        this->initialCamera = vtkSmartPointer<vtkCamera>::New();
        this->initialCamera->DeepCopy(ren->GetActiveCamera());

    }

}

void MainWindow::slotOpenCage(){

    if(!cageLoaded){

        QString filename = QFileDialog::getOpenFileName(nullptr,
                           "Choose a 3D cage",
                           QString::fromStdString(currentPath),
                           "PLY(*.ply);;STL(*.stl);;OBJ(*.obj);; VRML(.wrl);; IV(.iv);; OFF(.off);; EFF(.eff);; VERTRI(.tri)");

        if (!filename.isEmpty()){

            QFileInfo info(filename);
            cageFilename = info.fileName();
            currentPath = info.absolutePath().toStdString();
            std::cout << "loading: " << filename.toStdString() << std::endl << std::flush;
            this->cage = new DrawableMesh();
            if(cage->load(filename.toStdString().c_str()) != 0)
                this->write("Can't open cage file");

            else{

                this->lui->addCage(cageFilename, cage);

                this->cageLoaded = true;
                this->cage->setIsCage(true);
                this->cage->setDrawSurface(false);
                this->cage->setDrawWireframe(true);
                this->cage->setDrawPoints(true);
                this->selectionStyle = vtkSmartPointer<CageVerticesSelectionStyle>::New();
                this->cagePoints->SetPoints(this->cage->getPoints());
                vtkSmartPointer<vtkIdFilter> idFilterCage = vtkSmartPointer<vtkIdFilter>::New();
                idFilterCage->SetInputData(cagePoints);
                idFilterCage->PointIdsOn();
                idFilterCage->SetIdsArrayName("OriginalIds");
                idFilterCage->Update();
                vtkPolyData* inputCage = static_cast<vtkPolyData*>(idFilterCage->GetOutput());

                this->selectionStyle->SetCurrentRenderer(ren);
                this->selectionStyle->setPoints(inputCage);
                this->selectionStyle->setAssembly(this->meshAssembly);
                this->selectionStyle->setModel(this->model);
                this->selectionStyle->setCage(this->cage);
                this->selectionStyle->setAnnotationToCage(&(this->annotationToCage));
                this->selectionStyle->setSelectedPoints(&(this->pointsSelectionStatus));
                this->selectionStyle->setQvtkwidget(this->ui->qvtkWidget);
                this->selectionStyle->resetSelection();

                this->deformationStyle->setData(inputCage);
                this->deformationStyle->setModel(this->model);
                this->deformationStyle->setCage(this->cage);
                this->deformationStyle->setAssembly(this->meshAssembly);
                this->deformationStyle->setSelectedPoints(&(this->pointsSelectionStatus));
                this->deformationStyle->setCoordsComputed(false);
                this->deformationStyle->setQvtkwidget(this->ui->qvtkWidget);

                this->ui->actionVerticesSelection->setEnabled(true);
                this->ui->actionVerticesDeselection->setEnabled(true);
                this->ui->actionMeshDeformation->setEnabled(true);
                this->ui->actionStretch->setEnabled(true);
                this->ui->actionMean->setEnabled(true);
                this->ui->actionShowCage->setEnabled(true);
                this->ui->actionShowCage->setChecked(true);
                this->ui->cageVerticesLabel->setText(this->ui->cageVerticesLabel->text() + QString::number(cage->V.numels()));
                this->ui->cageEdgesLabel->setText(this->ui->cageEdgesLabel->text() + QString::number(cage->E.numels()));
                this->ui->cageTrianglesLabel->setText(this->ui->cageTrianglesLabel->text() + QString::number(cage->T.numels()));
            }

            cage->draw(meshAssembly);
            this->ui->qvtkWidget->update();

        }

    }


}

void MainWindow::slotOpenBC(){

    QString filename = QFileDialog::getOpenFileName(nullptr,
                       "Choose a BC file",
                       QString::fromStdString(currentPath),
                       "Coords(*.coord);; All(*.*)");

    if (!filename.isEmpty()){
        QFileInfo info(filename);
        currentPath = info.absolutePath().toStdString();
        ifstream loadfile;
        loadfile.open(filename.toStdString());
        std::string line;
        std::getline(loadfile, line);
        loadfile.close();
        this->modelToCage.clear();
        for(IMATI_STL::Node* n = this->model->V.head(); n != nullptr; n = n->next())
        {
            Vertex* v = static_cast<Vertex*>(n->data);
            this->modelToCage[this->model->getPointId(v)] = this->coords->getMaxInfluenceCageVertices(this->model->getPointId(v));
        }
        this->deformationStyle->setCoordsComputed(true);
        this->coordsComputed = true;
        this->lui->setBcComputed(true);
    }
}


void MainWindow::slotOpenFragment()
{
    QString filename = QFileDialog::getOpenFileName(nullptr,
                           "Choose a 3D model",
                           QString::fromStdString(currentPath),
                           "STL(*.stl);;PLY(*.ply);;OBJ(*.obj);; VRML(.wrl);; IV(.iv);; OFF(.off);; EFF(.eff);; VERTRI(.tri);; ALL(*)");

    if (!filename.isEmpty()){

        std::cout << "loading: " << filename.toStdString() << std::endl;
        DrawableMesh* newFragment = new DrawableMesh();

        if(newFragment->load(filename.toStdString().c_str()) != 0){
            this->write("Can't open mesh file");
        }else{
            QFileInfo info(filename);
            currentPath = info.absolutePath().toStdString();
            newFragment->setIsTemplate(false);
            newFragment->setIsCage(false);
            fragments.push_back(newFragment);
            fragmentsNames.push_back(info.fileName());
            this->lui->addFragment(info.fileName(), newFragment);
            newFragment->setDrawWireframe(false);
            meshAssembly->RemovePart(visualPropertiesAssembly);
            visualPropertiesAssembly = vtkSmartPointer<vtkPropAssembly>::New();
            newFragment->setMeshModified(true);
            newFragment->setAnnotationsModified(true);
            newFragment->update();
            newFragment->draw(meshAssembly);
            meshAssembly->AddPart(visualPropertiesAssembly);
            ren->Modified();
            this->ui->qvtkWidget->update();
        }
    }
}

void MainWindow::slotSaveFile(){

    QString filename = QFileDialog::getSaveFileName(nullptr,
                       "Save the deformed model",
                       ".",
                       "STL(*.stl);;PLY(*.ply);;OBJ(*.obj);; VRML(.wrl);; IV(.iv);; OFF(.off);; EFF(.eff);; VERTRI(.tri)");

    if (!filename.isEmpty()){

        string fname = filename.toStdString();

        QFileInfo info(filename);
        currentPath = info.absolutePath().toStdString();
        model->save(filename.toStdString().c_str());
        int pointPos = filename.lastIndexOf('.');
        filename.insert(pointPos, "_cage");
        cage->save(filename.toStdString().c_str());
        filename.truncate(pointPos);
        filename.append(".coord");
        coords->saveCoordinates(fname.substr(fname.find_last_of('/')));

    }

}

void MainWindow::slotSaveMesh(){

    QString filename = QFileDialog::getSaveFileName(nullptr,
                       "Save the mesh",
                       ".",
                       "STL(*.stl);;PLY(*.ply);;OBJ(*.obj);; VRML(.wrl);; IV(.iv);; OFF(.off);; EFF(.eff);; VERTRI(.tri)");

    if (!filename.isEmpty())
    {

        QFileInfo info(filename);
        currentPath = info.absolutePath().toStdString();
        model->save(filename.toStdString().c_str(),0);
    }

}

void MainWindow::slotSaveCage(){

    QString filename = QFileDialog::getSaveFileName(nullptr,
                       "Save the cage",
                       ".",
                       "STL(*.stl);;PLY(*.ply);;OBJ(*.obj);; VRML(.wrl);; IV(.iv);; OFF(.off);; EFF(.eff);; VERTRI(.tri)");

    if (!filename.isEmpty())
    {

        QFileInfo info(filename);
        currentPath = info.absolutePath().toStdString();
        cage->save(filename.toStdString().c_str(),0);
    }

}

void MainWindow::slotSaveBC(){

    QString filename = QFileDialog::getSaveFileName(nullptr,
                       "Save the Barycentric Coordinates",
                       ".",
                       "COORD(*.coord);;");

    if (!filename.isEmpty())
    {

        QFileInfo info(filename);
        currentPath = info.absolutePath().toStdString();
        coords->saveCoordinates(filename.toStdString().c_str());
    }
}


void MainWindow::slotClearMesh(){

    lui->removeMainMesh(model);
    clearMesh();

}

void MainWindow::slotClearCage(){
    lui->removeCage(cage);
    clearCage();
}

void MainWindow::slotClearConstraints()
{
    clearConstraints();
}

void MainWindow::slotClearCamera()
{
    resetCamera();
}

void MainWindow::slotClearSecondaryMeshes()
{

}

void MainWindow::slotChangeToDeformer(){

}

void MainWindow::slotChangeToAnnotator(){

}

void MainWindow::slotClose(){
    exit(0);
}

void MainWindow::slotGenerate()
{
    this->cage = new DrawableMesh(Utilities::generateCage(this->model, Utilities::VOLUMETRIC));
    this->cageLoaded = true;
    this->cage->setIsCage(true);
    this->cage->setDrawWireframe(true);
    this->cage->setDrawPoints(true);
    this->selectionStyle = vtkSmartPointer<CageVerticesSelectionStyle>::New();
    this->cagePoints->SetPoints(this->cage->getPoints());
    vtkSmartPointer<vtkIdFilter> idFilter = vtkSmartPointer<vtkIdFilter>::New();
    idFilter->SetInputData(cagePoints);
    idFilter->PointIdsOn();
    idFilter->SetIdsArrayName("OriginalIds");
    idFilter->Update();

    vtkPolyData* input = static_cast<vtkPolyData*>(idFilter->GetOutput());
    this->selectionStyle->SetCurrentRenderer(ren);
    this->selectionStyle->setPoints(input);
    this->selectionStyle->setAssembly(this->meshAssembly);
    this->selectionStyle->setCage(this->cage);
    this->selectionStyle->setSelectedPoints(&(this->pointsSelectionStatus));
    this->selectionStyle->setQvtkwidget(this->ui->qvtkWidget);
    this->selectionStyle->resetSelection();

    this->deformationStyle->setData(input);
    this->deformationStyle->setModel(this->model);
    this->deformationStyle->setCage(this->cage);
    this->deformationStyle->setAssembly(this->meshAssembly);
    this->deformationStyle->setSelectedPoints(&(this->pointsSelectionStatus));
    this->deformationStyle->setCoordsComputed(false);
    this->deformationStyle->setQvtkwidget(this->ui->qvtkWidget);

    this->ui->actionVerticesSelection->setEnabled(true);
    this->ui->actionVerticesDeselection->setEnabled(true);
    this->ui->actionMeshDeformation->setEnabled(true);
    this->ui->actionStretch->setEnabled(true);
    //this->ui->actionGreen->setEnabled(true);
    this->ui->actionMean->setEnabled(true);
    this->ui->actionShowCage->setEnabled(true);
    this->ui->actionShowCage->setChecked(true);
    this->ui->cageVerticesLabel->setText(this->ui->cageVerticesLabel->text() + QString::number(cage->V.numels()));
    this->ui->cageEdgesLabel->setText(this->ui->cageEdgesLabel->text() + QString::number(cage->E.numels()));
    this->ui->cageTrianglesLabel->setText(this->ui->cageTrianglesLabel->text() + QString::number(cage->T.numels()));

    cage->draw(meshAssembly);
    this->ui->qvtkWidget->update();
}

void MainWindow::slotComputeCoords(){

    this->computeCoords();
    this->deformationStyle->setCoords(coords);
    this->deformationStyle->setCoordsComputed(true);

    associateCageVerticesToAnnotations();
}

void MainWindow::slotShowMesh(bool value){

    model->setDrawable(value);
    model->draw(meshAssembly);
    this->ui->qvtkWidget->update();
}

void MainWindow::slotShowMeshSurface(bool value)
{
    model->setDrawSurface(value);
    model->draw(meshAssembly);
    this->ui->qvtkWidget->update();
}

void MainWindow::slotShowMeshPoints(bool value)
{
    model->setDrawPoints(value);
    model->draw(meshAssembly);
    this->ui->qvtkWidget->update();
}

void MainWindow::slotShowMeshAnnotations(bool value)
{
    model->setDrawAnnotations(value);
    model->draw(meshAssembly);
    this->ui->qvtkWidget->update();
}

void MainWindow::slotShowMeshWireframe(bool value)
{
    this->model->setDrawWireframe(value);
    model->draw(meshAssembly);
    this->ui->qvtkWidget->update();
}

void MainWindow::slotShowCage(bool value){

    cage->setDrawable(value);
    cage->draw(meshAssembly);
    this->ui->qvtkWidget->update();
}

void MainWindow::slotShowCageSurface(bool value)
{
    this->cage->setDrawSurface(value);
    cage->draw(meshAssembly);
    this->ui->qvtkWidget->update();
}

void MainWindow::slotShowCageWireframe(bool value)
{
    this->cage->setDrawWireframe(value);
    cage->draw(meshAssembly);
    this->ui->qvtkWidget->update();
}

void MainWindow::slotShowCagePoints(bool value)
{
    this->cage->setDrawPoints(value);
    cage->draw(meshAssembly);
    this->ui->qvtkWidget->update();
}

void MainWindow::slotLayerDialog(bool value)
{
    this->ui->line->setVisible(this->ui->line->isVisible() || value);
    this->ui->genericPalette->setVisible(true);
    this->lui->setVisible(value);
    this->ui->actionLayerDialog->setChecked(value);
    updateView();
}



void MainWindow::slotCheckConstraints()
{
    if(constrained){
        this->solver->checkConstraints();

        this->model->draw(meshAssembly);
        this->ui->qvtkWidget->update();
        this->updateView();

    }
}


void MainWindow::resetCamera()
{
    vtkSmartPointer<vtkCamera> newCamera = vtkSmartPointer<vtkCamera>::New();
    newCamera->SetPosition(initialCamera->GetPosition());
    newCamera->SetFocalPoint(initialCamera->GetFocalPoint());
    newCamera->SetViewUp(initialCamera->GetViewUp());
    ren->SetActiveCamera(newCamera);
    ren->ResetCamera();
    ren->GetRenderWindow()->Render();
    this->ui->qvtkWidget->update();
}

void MainWindow::clearAll()
{
    clear();
    clearMesh();
    clearCage();
    clearSelections();
    clearConstraints();
    clearCoordinates();
    clearSecondaryMeshes();
    clearCamera();
    vtkSmartPointer<vtkAreaPicker> picker = vtkSmartPointer<vtkAreaPicker>::New();
    this->ui->qvtkWidget->GetRenderWindow()->GetInteractor()->SetPicker(picker);
}

void MainWindow::clearMesh()
{
    this->meshLoaded = false;
    this->firstFragmentPositioned = false;
    this->lui->removeMainMesh(model);
    if(model != nullptr){
        this->meshAssembly->RemovePart(model->getCanvas());
        this->meshAssembly->Modified();
        delete model;
    }
    model = nullptr;
    modelCanvas = vtkSmartPointer<vtkPropAssembly>::New();
    this->modelTriangles = vtkSmartPointer<vtkPolyData>::New();
    this->pointsSelectionStatus.clear();
    this->ui->actionShowMesh->setChecked(true);
    this->ui->actionShowMeshSurface->setChecked(true);
    this->ui->actionShowMeshWireframe->setChecked(false);
    this->ui->actionShowMeshPoints->setChecked(false);
    this->ui->actionDelete->setChecked(false);
    this->ui->actionLineSelection->setChecked(false);
    this->ui->actionRectSelection->setChecked(false);
    this->ui->actionLassoSelection->setChecked(false);
    this->ui->verticesLabel->setText("Vertices: ");
    this->ui->edgesLabel->setText("Edges: ");
    this->ui->trianglesLabel->setText("Triangles: ");
    this->ui->qvtkWidget->update();
    clearSelections();
    clearCoordinates();
}

void MainWindow::clearCage()
{
    this->cageLoaded = false;
    this->firstFragmentPositioned = false;
    this->lui->removeCage(cage);
    if(this->cage != nullptr){
        this->meshAssembly->RemovePart(cage->getCanvas());
        delete this->cage;
    }
    this->cage = nullptr;
    this->lui->removeCage(cage);
    this->selectionStyle = vtkSmartPointer<CageVerticesSelectionStyle>::New();
    this->deformationStyle = vtkSmartPointer<MeshDeformationStyle>::New();
    this->pointsSelectionStatus.clear();
    this->ui->actionShowCage->setChecked(true);
    this->ui->actionShowCageSurface->setChecked(false);
    this->ui->actionShowCageWireframe->setChecked(true);
    this->ui->actionShowCagePoints->setChecked(true);
    this->ui->cageVerticesLabel->setText("Cage vertices: ");
    this->ui->cageEdgesLabel->setText("Cage edges: ");
    this->ui->cageTrianglesLabel->setText("Cage triangles: ");
    this->meshAssembly->Modified();
    this->ui->qvtkWidget->update();
    clearCoordinates();

}

void MainWindow::clearCoordinates()
{
    if(this->coords != nullptr)
        delete coords;
    coords = nullptr;
    this->coordsComputed = false;
    this->firstFragmentPositioned = false;
    this->lui->setBcComputed(false);
}

void MainWindow::clearSelections()
{
    this->ui->qvtkWidget->update();
}

void MainWindow::clearConstraints()
{
    this->constrained = false;
    this->deformationStyle->setConstrained(this->constrained);
}

void MainWindow::clearCamera()
{
    ren->RemoveAllViewProps();
    this->resetCamera();
    this->ui->qvtkWidget->update();
}

void MainWindow::clearSecondaryMeshes()
{
    for(unsigned int i = 0; i < fragments.size(); i++){
        DrawableMesh* mesh = fragments[i];
        fragments.erase(fragments.begin() + i);
        this->lui->removeFragment(fragments[i]);
        this->fragmentsNames.erase(fragmentsNames.begin() + i);
        delete mesh;
    }

    this->ui->qvtkWidget->update();
}

void MainWindow::associateCageVerticesToAnnotations()
{
    int flagValue = 12;
    for(IMATI_STL::Node* n = this->cage->V.head(); n != nullptr; n = n->next())
        static_cast<Vertex*>(n->data)->info = nullptr;

    for(unsigned int i = 0; i < this->model->getAnnotations().size(); i++)
    {
        std::vector<IMATI_STL::Vertex*> involved = this->model->getAnnotations()[i]->getInvolvedVertices();
        std::vector<unsigned int> associatedCageVertices;
        for(unsigned int j = 0; j < involved.size(); j++)
        {
            std::vector<unsigned int> cageVertices = this->coords->getMaxInfluenceCageVertices(this->model->getPointId(involved[j]));
            for(unsigned int k = 0; k < cageVertices.size(); k++)
            {
                Vertex* v = this->cage->getPoint(cageVertices[k]);
                if(v->info == nullptr)
                {
                    v->info = new int(flagValue);
                    associatedCageVertices.push_back(cageVertices[k]);
                }

            }
        }
        this->annotationToCage[i] = associatedCageVertices;

        for(unsigned int j = 0; j < associatedCageVertices.size(); j++)
            this->cage->getPoint(associatedCageVertices[j])->info = nullptr;
    }

    for(IMATI_STL::Node* n = this->cage->V.head(); n != nullptr; n = n->next())
        static_cast<Vertex*>(n->data)->info = nullptr;
}

void MainWindow::slotClearAll(){

    clearAll();
    init();
    drawInitialTetrahedron();

}

void MainWindow::slotModel(bool checked){

    if(!checked)
        model->setDrawable(false);
    else
        model->setDrawable(true);

    model->draw(meshAssembly);
    this->ui->qvtkWidget->update();

}

void MainWindow::slotCamera(bool checked){

    if(checked){

        vtkSmartPointer<vtkInteractorStyleTrackballCamera> cameraStyle = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();        iren->SetInteractorStyle(cameraStyle);
        this->ui->actionAddVerticesRectangle->setChecked(false);
        this->ui->actionRectSelection->setChecked(false);
        this->ui->actionLassoSelection->setChecked(false);
        this->ui->actionLineSelection->setChecked(false);
        this->ui->actionAddVerticesRectangle->setChecked(false);
        this->ui->actionVerticesSelection->setChecked(false);
        this->ui->actionVerticesDeselection->setChecked(false);
        this->ui->actionStretch->setChecked(false);
        this->ui->actionMeshDeformation->setChecked(false);

    }

}

void MainWindow::slotVerticesSelection(bool checked){

    if(checked){

        this->iren->SetInteractorStyle(selectionStyle);
        this->selectionStyle->SetCurrentRenderer(ren);
        this->selectionStyle->selectionMode = true;
        this->ui->actionAddVerticesRectangle->setChecked(false);
        this->ui->actionRectSelection->setChecked(false);
        this->ui->actionLassoSelection->setChecked(false);
        this->ui->actionLineSelection->setChecked(false);
        this->ui->actionAddVerticesRectangle->setChecked(false);
        this->ui->actionVerticesDeselection->setChecked(false);
        this->ui->actionStretch->setChecked(false);
        this->ui->actionMeshDeformation->setChecked(false);
        this->ui->actionCamera->setChecked(false);

    }else
        this->ui->actionVerticesSelection->setChecked(true);
}

void MainWindow::slotVerticesDeselection(bool checked){

    if(checked){

        iren->SetInteractorStyle(selectionStyle);
        selectionStyle->selectionMode = false;
        this->ui->actionAddVerticesRectangle->setChecked(false);
        this->ui->actionRectSelection->setChecked(false);
        this->ui->actionLassoSelection->setChecked(false);
        this->ui->actionLineSelection->setChecked(false);
        this->ui->actionAddVerticesRectangle->setChecked(false);
        this->ui->actionVerticesSelection->setChecked(false);
        this->ui->actionStretch->setChecked(false);
        this->ui->actionMeshDeformation->setChecked(false);
        this->ui->actionCamera->setChecked(false);

    }else
        this->ui->actionVerticesDeselection->setChecked(true);
}

void MainWindow::slotMeshDeformation(bool checked){

    if(checked){
        deformationStyle->setStretch(false);
        iren->SetInteractorStyle(deformationStyle);
        this->ui->actionAddVerticesRectangle->setChecked(false);
        this->ui->actionRectSelection->setChecked(false);
        this->ui->actionLassoSelection->setChecked(false);
        this->ui->actionLineSelection->setChecked(false);
        this->ui->actionAddVerticesRectangle->setChecked(false);
        this->ui->actionVerticesSelection->setChecked(false);
        this->ui->actionVerticesDeselection->setChecked(false);
        this->ui->actionStretch->setChecked(false);
        this->ui->actionCamera->setChecked(false);
    }

}

void MainWindow::slotStretch(bool checked)
{
    if(checked){
        deformationStyle->setStretch(true);
        this->ui->qvtkWidget->update();
        iren->SetInteractorStyle(deformationStyle);
        this->ui->actionCamera->setChecked(false);
        this->ui->actionAddVerticesRectangle->setChecked(false);
        this->ui->actionRectSelection->setChecked(false);
        this->ui->actionLassoSelection->setChecked(false);
        this->ui->actionLineSelection->setChecked(false);
        this->ui->actionAddVerticesRectangle->setChecked(false);
        this->ui->actionVerticesSelection->setChecked(false);
        this->ui->actionVerticesDeselection->setChecked(false);
        this->ui->actionMeshDeformation->setChecked(false);
    }
}

void MainWindow::computeCoords(){

    switch(coordType){
        case(GC):
            throw("At the moment it is not possible to use Green Coordinates.");
        case(MVC):
        default:
            this->coords = new MeanValueCoordinatesJu(model, cage);
    }

    this->coords->computeCoordinates();
    this->modelToCage.clear();
    for(IMATI_STL::Node* n = this->model->V.head(); n != nullptr; n = n->next())
    {
        Vertex* v = static_cast<Vertex*>(n->data);
        this->modelToCage[this->model->getPointId(v)] = this->coords->getMaxInfluenceCageVertices(this->model->getPointId(v));
    }
    this->coordsComputed = true;
    this->lui->setBcComputed(true);

}

void MainWindow::meshLoadedPreparation(){


    QFileInfo info(modelFilename);
    setWindowTitle(info.fileName() + " ― MainWindow");
    this->meshLoaded = true;
    this->model->setIsTemplate(true);
    this->model->setDrawSurface(true);
    this->model->setDrawWireframe(false);
    this->model->setDrawPoints(false);
    this->lui->addMainMesh(modelFilename, model);

    this->modelPoints->SetPoints(this->model->getPoints());
    this->modelTriangles->SetPoints(this->model->getPoints());
    this->modelTriangles->SetPolys(this->model->getTriangles());
    this->modelTriangles->BuildCells();
    this->modelTriangles->BuildLinks();

    vtkSmartPointer<vtkIdFilter> idFilter = vtkSmartPointer<vtkIdFilter>::New();
    idFilter->SetInputData(modelTriangles);
    idFilter->PointIdsOn();
    idFilter->SetIdsArrayName("OriginalIds");
    idFilter->Update();

    vtkSmartPointer<vtkIdFilter> idFilterMesh = vtkSmartPointer<vtkIdFilter>::New();
    idFilterMesh->SetInputData(modelPoints);
    idFilterMesh->PointIdsOn();
    idFilterMesh->SetIdsArrayName("OriginalMeshIds");
    idFilterMesh->Update();

    vtkSmartPointer<vtkPolyData> inputMesh = static_cast<vtkPolyData*>(idFilterMesh->GetOutput());
    vtkSmartPointer<vtkPolyData> input = static_cast<vtkPolyData*>(idFilter->GetOutput());

    this->ui->actionCamera->setEnabled(true);
    this->ui->actionDelete->setEnabled(true);
    this->ui->actionLineSelection->setEnabled(true);
    this->ui->actionRectSelection->setEnabled(true);
    this->ui->actionLassoSelection->setEnabled(true);
    this->ui->actionAnnotation->setEnabled(true);
    this->ui->actionAddVerticesRectangle->setEnabled(true);
    this->ui->actionShowMesh->setEnabled(true);
    this->ui->actionShowMesh->setChecked(true);
    this->ui->actionInfoPalette->setEnabled(true);
    this->ui->verticesLabel->setText("Vertices: " + QString::number(model->V.numels()));
    this->ui->edgesLabel->setText("Edges: " + QString::number(model->E.numels()));
    this->ui->trianglesLabel->setText("Triangles: " + QString::number(model->T.numels()));
    this->meshAssembly->RemovePart(visualPropertiesAssembly);
    this->visualPropertiesAssembly = vtkSmartPointer<vtkPropAssembly>::New();
    this->model->setMeshModified(true);
    this->model->setAnnotationsModified(true);
    this->model->update();
    this->model->draw(meshAssembly);
    this->meshAssembly->AddPart(visualPropertiesAssembly);
    this->ren->AddActor(meshAssembly);
    this->ren->Modified();
    this->ui->qvtkWidget->update();
    double corner[3], min[3], mid[3], max[3], sizes[3];
    vtkOBBTree::ComputeOBB(this->model->getPoints(), corner, max, mid, min, sizes);

    ExtendedTrimesh* obb = new ExtendedTrimesh();
    IMATI_STL::Point minAx(min[0], min[1] , min[2] );
    IMATI_STL::Point midAx(mid[0], mid[1] , mid[2] );
    IMATI_STL::Point maxAx(max[0], max[1] , max[2] );
    Vertex* d = obb->newVertex(corner[0], corner[1], corner[2]);
    Vertex* c = obb->newVertex();
    c->setValue(*d + minAx);
    Vertex* e = obb->newVertex();
    e->setValue(*d + midAx);
    Vertex* a = obb->newVertex();
    a->setValue(*d + maxAx);
    Vertex* b = obb->newVertex();
    b->setValue(*a + minAx);
    Vertex* f = obb->newVertex();
    f->setValue(*a + midAx);
    Vertex* g = obb->newVertex();
    g->setValue(*b + midAx);
    Vertex* h = obb->newVertex();
    h->setValue(*c + midAx);
    obb->V.appendTail(a);
    obb->V.appendTail(b);
    obb->V.appendTail(c);
    obb->V.appendTail(d);
    obb->V.appendTail(e);
    obb->V.appendTail(f);
    obb->V.appendTail(g);
    obb->V.appendTail(h);
    ExtVertex* a_ = new ExtVertex(a);
    ExtVertex* b_ = new ExtVertex(b);
    ExtVertex* c_ = new ExtVertex(c);
    ExtVertex* d_ = new ExtVertex(d);
    ExtVertex* e_ = new ExtVertex(e);
    ExtVertex* f_ = new ExtVertex(f);
    ExtVertex* g_ = new ExtVertex(g);
    ExtVertex* h_ = new ExtVertex(h);
    obb->CreateTriangleFromVertices(a_, d_, f_);
    obb->CreateTriangleFromVertices(a_, f_, g_);
    obb->CreateTriangleFromVertices(a_, g_, b_);
    obb->CreateTriangleFromVertices(a_, b_, d_);
    obb->CreateTriangleFromVertices(b_, c_, d_);
    obb->CreateTriangleFromVertices(b_, h_, c_);
    obb->CreateTriangleFromVertices(b_, g_, h_);
    obb->CreateTriangleFromVertices(c_, e_, d_);
    obb->CreateTriangleFromVertices(c_, h_, e_);
    obb->CreateTriangleFromVertices(d_, e_, f_);
    obb->CreateTriangleFromVertices(e_, h_, f_);
    obb->CreateTriangleFromVertices(f_, h_, g_);
    obb->mergeCoincidentEdges();
    obb->checkGeometry();
    obb->checkConnectivity();
    obb->savePLY("teapot_bb_cage.ply");
    ren->ResetCamera();
    this->totalWidth = ((*c) - (*d)).length();
    this->totalHeight = ((*a) - (*d)).length();
    this->totalDepth = ((*e) - (*d)).length();
}

void MainWindow::slotMeanValue(){
    coordType = MVC;
}

void MainWindow::slotGreen(){
    coordType = GC;
}

void MainWindow::slotVisible(bool checked){

    this->selectionStyle->setVisiblePointsOnly(checked);

}

void MainWindow::slotMeshColor(){

    int r,g,b;
    QColor meshColor = QColorDialog::getColor();
    meshColor.getRgb(&r,&g,&b);
    this->model->setColor(r,g,b);
    this->model->draw(meshAssembly);
    this->ui->qvtkWidget->update();
}

void MainWindow::slotCageColor(){

    int r,g,b;
    QColor cageColor = QColorDialog::getColor();
    cageColor.getRgb(&r,&g,&b);
    this->cage->setColor(r,g,b);
    this->cage->draw(meshAssembly);
    this->ui->qvtkWidget->update();

}

void MainWindow::slotAddAnnotationsConstraint(std::string type, double weight, double minValue, double maxValue, unsigned int measureId1, unsigned int measureId2, bool directed)
{
    AnnotationsConstraint* constraint;
    if(type.compare("Surfaces same measure") == 0){
        Annotation* annotation1 = cd->getSubjects()[0];
        Annotation* annotation2 = cd->getSubjects()[1];
        Attribute* attribute1 = annotation1->getAttributes()[measureId1];
        Attribute* attribute2 = annotation2->getAttributes()[measureId2];
        constraint = new AnnotationMeasuresConstraint();
        GeometricAttribute* geometricAttribute1 = dynamic_cast<GeometricAttribute*>(attribute1);
        GeometricAttribute* geometricAttribute2 = dynamic_cast<GeometricAttribute*>(attribute2);
        dynamic_cast<AnnotationMeasuresConstraint*>(constraint)->setAttribute1(geometricAttribute1);
        dynamic_cast<AnnotationMeasuresConstraint*>(constraint)->setAttribute2(geometricAttribute2);
    }else if(type.compare("Surface same measure") == 0){
        if(measureId1 == measureId2){
            std::cerr << "Error: constraining a measure to be equal to itself is not sensible" << std::endl << std::flush;
            return;
        }
        Annotation* annotation = cd->getSubjects()[0];
        Attribute* attribute1 = annotation->getAttributes()[measureId1];
        Attribute* attribute2 = annotation->getAttributes()[measureId2];
        constraint = new AnnotationMeasuresConstraint();
        GeometricAttribute* geometricAttribute1 = dynamic_cast<GeometricAttribute*>(attribute1);
        GeometricAttribute* geometricAttribute2 = dynamic_cast<GeometricAttribute*>(attribute2);
        dynamic_cast<AnnotationMeasuresConstraint*>(constraint)->setAttribute1(geometricAttribute1);
        dynamic_cast<AnnotationMeasuresConstraint*>(constraint)->setAttribute2(geometricAttribute2);
    } else
        constraint = new AnnotationsConstraint();
    constraint->setId(reachedContraintId++);
    constraint->setAnnotations(cd->getSubjects());
    constraint->setType(type);
    constraint->setWeight(weight);
    constraint->setMinValue(minValue);
    constraint->setMaxValue(maxValue);
    constraint->constrain();
    semanticConstraints.push_back(constraint);

    for (unsigned int i = 0; i < constraint->getAnnotations().size(); i++) {
        for (unsigned int j = i; j < constraint->getAnnotations().size(); j++) {
            if( i == j && constraint->getAnnotations().size() != 1)
                continue;
            model->addAnnotationsRelationship(constraint->getAnnotations()[i], constraint->getAnnotations()[j], type, weight, directed);
        }
    }

    this->model->setMeshModified(true);
    this->model->update();
    this->model->draw(this->meshAssembly);
    this->ui->qvtkWidget->update();
}

void MainWindow::slotConstrainRelationship(AnnotationsRelationship *&relationship)
{
    AnnotationsRelationship* r = relationship;
    AnnotationsConstraint* constraint = new AnnotationsConstraint(relationship);
    constraint->constrain();
    std::cout << constraint << " " << relationship << std::endl << std::flush;
    relationship = constraint;
    delete r;
    semanticConstraints.push_back(constraint);
    this->model->setMeshModified(true);
    this->model->update();
    this->model->draw(this->meshAssembly);
    this->ui->qvtkWidget->update();
}


void MainWindow::slotConstrainSystem(std::vector<AnnotationsConstraint*> semanticConstraints)
{
    this->solver = new ConstraintSolver();
    this->semanticConstraints = semanticConstraints;
    this->constrained = true;
    this->lui->setConstraintsImposed(true);
    this->deformationStyle->setConstrained(constrained);
    this->deformationStyle->setSolver(this->solver);
    this->deformationStyle->setSemanticConstraints(semanticConstraints);
}

void MainWindow::slotReleaseSystem()
{
    if(this->solver != nullptr)
        delete this->solver;
    this->constrained = false;
    this->lui->setConstraintsImposed(false);
    this->deformationStyle->setConstrained(constrained);
}

void MainWindow::slotSubstituteMesh(DrawableMesh *original, DrawableMesh *newMesh, QString filename)
{
    meshAssembly->RemovePart(original->getCanvas());
    if(original == model)
    {
        lui->removeMainMesh(original);
        delete original;
        model = newMesh;
        this->meshLoadedPreparation();
        this->ui->verticesLabel->setText(QString::fromStdString("Vertices: ") + QString::number(model->V.numels()));
        this->ui->edgesLabel->setText(QString::fromStdString("Edges: ") + QString::number(model->E.numels()));
        this->ui->trianglesLabel->setText(QString::fromStdString("Triangles: ") + QString::number(model->T.numels()));
    }
    else
    {
        lui->removeFragment(original);
        unsigned int pos = static_cast<unsigned int>(std::find(fragments.begin(), fragments.end(), original) - fragments.begin());
        fragments[pos] = newMesh;
        fragmentsNames[pos] = filename;
        this->meshAssembly->RemovePart(original->getCanvas());
        this->meshAssembly->Modified();
        this->ren->Modified();
        this->ui->qvtkWidget->update();
        this->updateView();
        delete original;
        lui->addFragment(filename, newMesh);

        newMesh->setMeshModified(true);
        newMesh->setAnnotationsModified(true);
        newMesh->update();
        newMesh->draw(meshAssembly);
    }
    meshAssembly->RemovePart(newMesh->getCanvas()); //This because in AnnotationWindow the canvas is overridden just after.
    resetCamera();
}

void MainWindow::slotUpdateMeshView(DrawableMesh * mesh)
{
    mesh->draw(meshAssembly);
    ren->Modified();
    this->ui->qvtkWidget->update();
}

void MainWindow::slotDeleteMesh(DrawableMesh *mesh)
{
    if(mesh == model)
        clearMesh();
    else if(mesh == cage)
        clearCage();
    else{
        unsigned int pos = static_cast<unsigned int>(std::find(fragments.begin(), fragments.end(), mesh) - fragments.begin());
        fragments.erase(fragments.begin() + pos);
        fragmentsNames.erase(fragmentsNames.begin() + pos);
        this->meshAssembly->RemovePart(mesh->getCanvas());
        delete mesh;
        this->meshAssembly->Modified();
        this->ren->Modified();
        this->ui->qvtkWidget->update();
        this->updateView();
    }
}

void MainWindow::slotEditAnnotations(DrawableMesh *mesh)
{
    meshAssembly->RemovePart(mesh->getCanvas());
    if(ren != nullptr && mesh != nullptr)
    {
        for(unsigned int i = 0; i < mesh->getAnnotations().size(); i++)
            for(unsigned int j = 0; j < mesh->getAnnotations()[i]->getAttributes().size(); j++)
                meshAssembly->RemovePart(dynamic_cast<DrawableAttribute*>(mesh->getAnnotations()[i]->getAttributes()[j])->getCanvas());

        meshAssembly->Modified();
        this->ui->qvtkWidget->update();
    }
    AnnotationWindow* annWin = new AnnotationWindow(this);
    annWin->setCurrentPath(currentPath);
    annWin->setIsSystemConstrained(this->constrained);
    annWin->setCoordsComputed(coordsComputed);
    annWin->setMesh(mesh);
    annWin->setSemanticConstraints(semanticConstraints);
    annWin->show();
    connect(annWin, SIGNAL(annotationWindowClosed(DrawableMesh*, std::vector<AnnotationsConstraint*>)), this, SLOT(slotAnnotationWindowClosed(DrawableMesh*, std::vector<AnnotationsConstraint*>)));
    connect(annWin, SIGNAL(constrainSystem(std::vector<AnnotationsConstraint*>)), this, SLOT(slotConstrainSystem(std::vector<AnnotationsConstraint*>)));
    connect(annWin, SIGNAL(releaseConstraints()), this, SLOT(slotReleaseSystem()));
    connect(annWin, SIGNAL(constrainRelationship(AnnotationsRelationship*&)), this, SLOT(slotConstrainRelationship(AnnotationsRelationship*&)));
    connect(annWin, SIGNAL(substituteMesh(DrawableMesh*, DrawableMesh*, QString)), this, SLOT(slotSubstituteMesh(DrawableMesh*, DrawableMesh*, QString)));
    connect(annWin, SIGNAL(currentPathChanged(std::string)), this, SLOT(slotCurrentPathChanged(std::string)));
}

void MainWindow::slotFitRigidly(DrawableMesh * mesh)
{
    CorrespondencesDialog* cd = new CorrespondencesDialog(this);
    cd->setTemplateMesh(model);
    cd->setFragmentMesh(mesh);
    cd->updateView();
    cd->exec();
    std::vector<std::pair<int, int> > correspondences = cd->getCorrespondences();
    delete cd;

    if(correspondences.size() > 0){
        //Rigid registration component: Umeyama algorithm
        Eigen::Matrix3Xd src, tgt;
        src.resize(3, static_cast<long>(correspondences.size()));
        tgt.resize(3, static_cast<long>(correspondences.size()));
        for(unsigned int i = 0; i < correspondences.size(); i++){
            Vertex* srcPt = model->getPoint(static_cast<unsigned long>(correspondences[i].first));
            Vertex* tgtPt = mesh->getPoint(static_cast<unsigned long>(correspondences[i].second));
            src.col(i) = Eigen::Vector3d(srcPt->x, srcPt->y, srcPt->z);
            tgt.col(i) = Eigen::Vector3d(tgtPt->x, tgtPt->y, tgtPt->z);
        }
        Eigen::Matrix4d transformation = Eigen::umeyama(src, tgt, !firstFragmentPositioned);
        if(!firstFragmentPositioned && fitTemplateOnFragments)
        {
            for(unsigned int i = 0; i < model->V.numels(); i++){
                IMATI_STL::Vertex* v = model->getPoint(i);
                Eigen::Vector4d tmp = {v->x, v->y, v->z, 1};
                tmp = transformation * tmp;
                v->setValue(tmp(0), tmp(1), tmp(2));
            }
            this->model->setMeshModified(true);
            this->model->update();
            this->model->draw(meshAssembly);
            this->cage->setMeshModified(true);
            this->cage->update();
            this->cage->draw(meshAssembly);
            firstFragmentPositioned = true;
        } else {
            for(unsigned int i = 0; i < mesh->V.numels(); i++){
                IMATI_STL::Vertex* v = mesh->getPoint(i);
                Eigen::Vector4d tmp = {v->x, v->y, v->z, 1};
                tmp = transformation * tmp;
                v->setValue(tmp(0), tmp(1), tmp(2));
            }
            mesh->setMeshModified(true);
            mesh->update();
            mesh->draw(this->meshAssembly);
        }

        this->ui->qvtkWidget->update();
        this->meshAssembly->Modified();
    }
    solver->checkConstraints();
}


void MainWindow::slotAnnotationWindowClosed(DrawableMesh *mesh, std::vector<AnnotationsConstraint*> semanticConstraints)
{
    this->semanticConstraints = semanticConstraints;

    meshAssembly->RemovePart(mesh->getCanvas());
    for(unsigned int i = 0; i < mesh->getAnnotations().size(); i++)
        for(unsigned int j = 0; j < mesh->getAnnotations()[i]->getAttributes().size(); j++)
        {
            Attribute* a = mesh->getAnnotations()[i]->getAttributes()[j];
            meshAssembly->RemovePart(dynamic_cast<DrawableAttribute*>(a)->getCanvas());
            dynamic_cast<DrawableAttribute*>(a)->setRenderer(ren);
        }

    mesh->draw(meshAssembly);
    if(coordsComputed)
        associateCageVerticesToAnnotations();

    this->ui->qvtkWidget->update();
}

void MainWindow::slotCurrentPathChanged(string newPath)
{
    currentPath = newPath;
}

void MainWindow::on_showBoundingBox_stateChanged(int value)
{
    this->showBoundingBox = value;
    this->updateView();
}

BarycentricCoordinates *MainWindow::getCoords() const
{
    return coords;
}

void MainWindow::setCoords(BarycentricCoordinates *value)
{
    coords = value;
}
