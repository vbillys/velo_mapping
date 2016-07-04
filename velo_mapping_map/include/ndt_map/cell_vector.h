#ifndef LSL_CELL_VECTOR_HH
#define LSL_CELL_VECTOR_HH

#include <ndt_map/spatial_index.h>
#include <ndt_map/ndt_cell.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/console/print.h>

namespace lslgeneric
{

/** \brief A spatial index represented as a grid map
    \details A grid map with delayed allocation of cells.
*/
class CellVector : public SpatialIndex
{
public:
    CellVector();
    CellVector(NDTCell* cellPrototype);
    CellVector(const CellVector& other);
    virtual ~CellVector();

    virtual NDTCell* getCellForPoint(const pcl::PointXYZ &point);
    virtual NDTCell* addPoint(const pcl::PointXYZ &point);
    void addCellPoints(pcl::PointCloud<pcl::PointXYZ> pc, const std::vector<size_t> &indices);
    
    void addCell(NDTCell* cell);
    void addNDTCell(NDTCell* cell);

    virtual typename SpatialIndex::CellVectorItr begin();
    virtual typename SpatialIndex::CellVectorItr end();
    virtual int size();

    ///clone - create an empty object with same type
    virtual SpatialIndex* clone() const;
    ///copy - create the same object as a new instance
    virtual SpatialIndex* copy() const;

    ///method to return all cells within a certain radius from a point
    virtual void getNeighbors(const pcl::PointXYZ &point, const double &radius, std::vector<NDTCell*> &cells);

    ///sets the cell factory type
    virtual void setCellType(NDTCell *type);


    void initKDTree();

    NDTCell* getClosestNDTCell(const pcl::PointXYZ &pt);
    std::vector<NDTCell*> getClosestNDTCells(const pcl::PointXYZ &point, double &radius);
    NDTCell* getCellIdx(unsigned int idx) const;

    void cleanCellsAboveSize(double size);
    int loadFromJFF(FILE * jffin);
private:
    std::vector<NDTCell*> activeCells;
    NDTCell *protoType;
    pcl::KdTreeFLANN<pcl::PointXYZ> meansTree;
    typename pcl::KdTree<pcl::PointXYZ>::PointCloudPtr mp;
    bool treeUpdated;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}; //end namespace
//#include <ndt_map/impl/cell_vector.hpp>

#endif
