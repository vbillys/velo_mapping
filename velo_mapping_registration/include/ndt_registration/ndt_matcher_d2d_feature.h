#ifndef NDTMATCHERFEATUREF2F_HH
#define NDTMATCHERFEATUREF2F_HH

#include <ndt_registration/ndt_matcher_d2d.h>
namespace lslgeneric
{
/**
 * This class implements NDT / NDT registration with a priory known correspondances.
 */
class NDTMatcherFeatureD2D : public lslgeneric::NDTMatcherD2D
{
public:
    NDTMatcherFeatureD2D(const std::vector<std::pair<int, int> > &corr, double trimFactor = 1.) : _corr(corr), _trimFactor(trimFactor)
    {
        _goodCorr.resize(corr.size());
        std::fill(_goodCorr.begin(), _goodCorr.end(), true);
    }

    /**
     * Registers a point cloud to an NDT structure.
     * \param  target
     *   Reference data.
     * \param  source
     *   The output transformation registers this point cloud to \c target.
     * \param  T
     *   This is an input/output parameter. The initial value of \c T
     *   gives the initial pose estimate of \c source. When the
     *   algorithm terminates, \c T holds the registration result.
    bool match( lslgeneric::NDTMap<PointTarget>& target,
          lslgeneric::NDTMap<PointSource>& source,
          Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T);
     */
    /**
     * computes the covariance of the match between moving and fixed, at T.
     * result is returned in cov
     */
    bool covariance( lslgeneric::NDTMap& target,
                     lslgeneric::NDTMap& source,
                     Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T,
                     Eigen::Matrix<double,6,6> &cov
                   );

    //compute the score of a point cloud to an NDT
    virtual double scoreNDT(std::vector<lslgeneric::NDTCell*> &source,
                            lslgeneric::NDTMap &target,
                            Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T);

    //compute the score gradient & hessian of a point cloud + transformation to an NDT
    // input: moving, fixed, tr, computeHessian
    //output: score_gradient, Hessian
    virtual double derivativesNDT(
	    const std::vector<NDTCell*> &sourceNDT,
	    const NDTMap &targetNDT,
	    Eigen::MatrixXd &score_gradient,
	    Eigen::MatrixXd &Hessian,
	    bool computeHessian
    );
#if 0
    virtual bool update_gradient_hessian(
        Eigen::Matrix<double,6,1> &score_gradient,
        Eigen::Matrix<double,6,6> &Hessian,

        Eigen::Vector3d &m1,
        Eigen::Matrix3d &C1);
#endif

    using NDTMatcherD2D::Jest;
    using NDTMatcherD2D::Hest;
    using NDTMatcherD2D::Zest;
    using NDTMatcherD2D::ZHest;
    using NDTMatcherD2D::lfd1;
    using NDTMatcherD2D::lfd2;
    using NDTMatcherD2D::normalizeAngle;
    using NDTMatcherD2D::NUMBER_OF_ACTIVE_CELLS;
protected:
    const std::vector<std::pair<int, int> > & _corr;
    double _trimFactor;
    std::vector<bool> _goodCorr;
};
} // namespace


#endif
