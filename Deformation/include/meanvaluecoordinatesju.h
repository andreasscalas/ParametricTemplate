#ifndef MEANVALUECOORDINATESJU_H
#define MEANVALUECOORDINATESJU_H

#include <barycentriccoordinates.h>
#include <drawablemesh.h>

#include <Eigen/Sparse>

/**
 * @brief The MeanValueCoordinatesJu class Class that allows the deformation of a model given a cage,
 * using the mathematical structure defined by Ju et al. in the paper "Mean Value Coordinates for Closed Triangular Meshes".
 */
class MeanValueCoordinatesJu : public BarycentricCoordinates{

    private:
        DrawableMesh* model;                                //The model to deform
        DrawableMesh* cage;                                 //The cage with which the model is deformed
        Eigen::MatrixX3d cageVertices;                      //The cage with which the model is deformed
        Eigen::SparseMatrix<double, Eigen::ColMajor> coordinates;
        std::vector<std::vector<double> > coords;           //The coordinates linked to the vertices
        std::string coordsFileName;                             //Name of the file in which the coordinates are saved
        static const unsigned int NUM_THREADS = 10;                //Number of threads which will be executed in parallel

        Eigen::SparseMatrix<bool, Eigen::RowMajor> maxInfluenceValues;
        double threshold;

        void analyseMaxInfluenceThreshold();
    public:

        /**
        * @brief MeanValueCoordinatesJu::MeanValueCoordinatesJu Main constructor of the class. It requires the
        * model which will be deformed and the cage used for that purpose.
        * @param model the model which will be deformed
        * @param cage the cage used for the deformation
        */
        MeanValueCoordinatesJu(DrawableMesh* model, DrawableMesh* cage);

        ~MeanValueCoordinatesJu();
        /**
         * @brief MeanValueCoordinatesJu::computeCoordinates Method that manages the computation of the coordinates
         */
        void computeCoordinates();

        /**
         * @brief MeanValueCoordinatesJu::saveCoordinates Method that manages the saving of the computed coordinates
         * @param filename the name of the file in which the coordinates will be saved
         */
        void saveCoordinates(std::string filename);

        /**
         * @brief MeanValueCoordinatesJu::loadCoordinates Method that manages the loading of the coordinates
         * @param filename the name of the file from which the coordinates will be loaded
         */
        void loadCoordinates(std::string filename);

        /**
         * @brief MeanValueCoordinatesJu::deform Method that manages the deformation of the model.
         */
        void deform();

        /**
         * @brief MeanValueCoordinatesJu::getCoordinates Method that allow the gathering of the coordinates
         * @return the coordinates set
         */
        std::vector<std::vector<double> > getCoordinates();

        double getThreshold() const;

        void setThreshold(double value);

        virtual std::vector<unsigned int> getMaxInfluenceCageVertices(unsigned int) override;

        virtual void updateCageVertexPosition(unsigned int, Eigen::Vector3d) override;
};

#endif // MEANVALUECOORDINATESJU_H
