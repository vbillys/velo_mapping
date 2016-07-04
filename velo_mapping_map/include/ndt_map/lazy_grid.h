#ifndef LSL_LAZZY_GRID_HH
#define LSL_LAZZY_GRID_HH

#include <ndt_map/spatial_index.h>
#include <ndt_map/ndt_cell.h>

namespace lslgeneric
{

/** \brief A spatial index represented as a grid map
    \details A grid map with delayed allocation of cells.
*/
class LazyGrid : public SpatialIndex
{
public:
    LazyGrid(double cellSize);
    LazyGrid(LazyGrid *prot);
    LazyGrid(double sizeXmeters, double sizeYmeters, double sizeZmeters,
             double cellSizeX, double cellSizeY, double cellSizeZ,
             double _centerX, double _centerY, double _centerZ,
             NDTCell *cellPrototype );
    virtual ~LazyGrid();

    virtual NDTCell* getCellForPoint(const pcl::PointXYZ &point);
    virtual NDTCell* addPoint(const pcl::PointXYZ &point);

    //these two don't make much sense...
    ///iterator through all cells in index, points at the begining
    virtual typename SpatialIndex::CellVectorItr begin();
    ///iterator through all cells in index, points at the end
    virtual typename SpatialIndex::CellVectorItr end();
    virtual int size();

    ///clone - create an empty object with same type
    virtual SpatialIndex* clone() const;
    virtual SpatialIndex* copy() const;

    ///method to return all cells within a certain radius from a point
    virtual void getNeighbors(const pcl::PointXYZ &point, const double &radius, std::vector<NDTCell*> &cells);

    ///sets the cell factory type
    virtual void setCellType(NDTCell *type);

    virtual void setCenter(const double &cx, const double &cy, const double &cz);
    virtual void setSize(const double &sx, const double &sy, const double &sz);

    virtual NDTCell* getClosestNDTCell(const pcl::PointXYZ &pt, bool checkForGaussian=true);
    virtual std::vector<NDTCell*> getClosestNDTCells(const pcl::PointXYZ &pt, int &n_neigh, bool checkForGaussian=true);
    virtual std::vector<NDTCell*> getClosestCells(const pcl::PointXYZ &pt);

    virtual inline void getCellAt(int indX, int indY, int indZ, NDTCell* &cell){
	if(indX < sizeX && indY < sizeY && indZ < sizeZ && indX >=0 && indY >=0 && indZ >=0){
	    cell = dataArray[indX][indY][indZ];
	}else{
		cell = NULL;
		
	}
    }
    virtual inline void getCellAt(const pcl::PointXYZ& pt, NDTCell* &cell){
	int indX,indY,indZ;
	this->getIndexForPoint(pt,indX,indY,indZ);
	this->getCellAt(indX,indY,indZ,cell);
    }
    //FIXME: these two are now not needed anymore
    virtual inline void getNDTCellAt(int indX, int indY, int indZ, NDTCell* &cell){
			if(indX < sizeX && indY < sizeY && indZ < sizeZ && indX >=0 && indY >=0 && indZ >=0){
					cell = (dataArray[indX][indY][indZ]);
			}else{
				cell = NULL;
			}
    }
    virtual inline void getNDTCellAt(const pcl::PointXYZ& pt, NDTCell* &cell){
			int indX,indY,indZ;
			this->getIndexForPoint(pt,indX,indY,indZ);
			this->getNDTCellAt(indX,indY,indZ,cell);
    }

    void getCellSize(double &cx, double &cy, double &cz);
    void getGridSize(int &cx, int &cy, int &cz);
    void getGridSizeInMeters(double &cx, double &cy, double &cz);
    void getCenter(double &cx, double &cy, double &cz);
    virtual void getIndexForPoint(const pcl::PointXYZ& pt, int &idx, int &idy, int &idz);
    NDTCell* getProtoType()
    {
        return protoType;
    }

    virtual void initialize();
    virtual void initializeAll() ;

    NDTCell ****getDataArrayPtr()
    {
        return dataArray;
    }

    ///reads map contents from .jff file
    virtual int loadFromJFF(FILE * jffin);
    bool traceLine(const Eigen::Vector3d &origin, const pcl::PointXYZ &endpoint, const Eigen::Vector3d &diff, const double& maxz, std::vector<NDTCell*> &cells);
    bool traceLineWithEndpoint(const Eigen::Vector3d &origin, const pcl::PointXYZ &endpoint, const Eigen::Vector3d &diff, const double& maxz, std::vector<NDTCell*> &cells, Eigen::Vector3d &final_point);
    bool isInside(const pcl::PointXYZ& pt) {
			int indX,indY,indZ;
			this->getIndexForPoint(pt,indX,indY,indZ);
			return(indX < sizeX && indY < sizeY && indZ < sizeZ && indX >=0 && indY >=0 && indZ >=0);
    }
protected:
    bool initialized;
    NDTCell ****dataArray;
    //bool ***linkedCells;
    NDTCell *protoType;
    std::vector<NDTCell*> activeCells;
    bool centerIsSet, sizeIsSet;

    double sizeXmeters, sizeYmeters, sizeZmeters;
    double cellSizeX, cellSizeY, cellSizeZ;
    double centerX, centerY, centerZ;
    int sizeX,sizeY,sizeZ;

    virtual bool checkCellforNDT(int indX, int indY, int indZ, bool checkForGaussian=true);
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}; //end namespace
//#include<ndt_map/impl/lazy_grid.hpp>

#endif
