#ifndef __NUMERIC_PLANEFITTING_HPP__
#define __NUMERIC_PLANEFITTING_HPP__

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

namespace numeric
{

/** 
 * Performs a linear least squares regression of a plane to a set of points in
 * 3D space. The points are added incrementally using the update method, and can
 * be weighted according to their contribution to the fit. 
 * A call to getCoeffs() will solve the regression and return the coefficients 
 * of the fitted plane. Note that the linear least squares fit is over the
 * distance on the z-axis, and not actually the shortest distance to the plane.
 */
template<class Scalar>
class PlaneFitting
{
public:
    typedef typename Eigen::Matrix<Scalar,3,1> Vector3;
    typedef typename Eigen::Matrix<Scalar,3,3> Matrix3;

    Scalar x, y, z, xx, yy, xy, xz, yz, zz, n;

    PlaneFitting() :
	x(0), y(0), z(0), xx(0), yy(0), xy(0), xz(0), yz(0), zz(0), n(0) {}

    explicit PlaneFitting( const Vector3& p, Scalar weight = 1.0 ) :
        x(p.x() * weight),
        y(p.y() * weight),
        z(p.z() * weight),
        xx(p.x()*x),
        yy(p.y()*y),
        xy(p.x()*y),
        xz(p.x()*z),
        yz(p.y()*z),
        zz(p.z()*z),
        n(weight)
    {
    }

    /** 
     * @brief scale the statistics
     * Note that this will not have influence on the solution,
     * but will only change the relative weighting towards additional datums.
     */
    void scale( Scalar scale )
    {
	x  *= scale;
	y  *= scale;
	z  *= scale;
	xx *= scale;
	yy *= scale;
	xy *= scale;
	xz *= scale;
	yz *= scale;
	zz *= scale;
	n  *= scale;
    }

    /**
     * @brief clears all previous input to the update method 
     */
    void clear()
    {
	x = y = z = xx = yy = xy = xz = yz = zz = n = 0;
    }

    void update( const PlaneFitting& other )
    {
	x += other.x;
	y += other.y;
	z += other.z;
	xx += other.xx;
	yy += other.yy;
	xy += other.xy;
	xz += other.xz;
	yz += other.yz;
	zz += other.zz;
	n += other.n;
    }

    void update( const Vector3& p, Scalar weight = 1.0 )
    {
	update( PlaneFitting( p, weight ) );
    }

    class Result
    {
	Eigen::LDLT<Matrix3> ldlt;
	Vector3 coeffs;
	Scalar res;

    public:
	explicit Result( const PlaneFitting<Scalar>& sum )
	{
	    Matrix3 A;
	    Vector3 b;
	    A << 
	      sum.xx, sum.xy, sum.x,
	      sum.xy, sum.yy, sum.y,
	      sum.x, sum.y, sum.n;

	    b = Vector3( sum.xz, sum.yz, sum.z );

	    ldlt.compute( A );
	    coeffs = ldlt.solve( b );
	    res = sum.zz - b.dot(coeffs); // == sum.zz - 2*b^T*coeffs + coeffs^T*A*coeffs
	}

	const Vector3& getCoeffs() const
	{
	    return coeffs;
	}

	Scalar getResiduals() const
	{
	    return res;
	}

	Matrix3 getCovariance() const
	{
	    Matrix3 cov = 
		getResiduals() * ldlt.solve( Matrix3::Identity() );
	    return cov;
	}
    };

    /**
     * @brief Solve the regression and return a result object  
     *
     * the result object can be queried for different aspects
     * like coefficients, residuals or covariance matrix.
     *
     * @result result object of the regression
     */
    Result solve() const
    {
	return Result( *this );
    }

    /**
     * @brief Get the coefficients of the fitted plane. 
     *
     * will return all zeros if no input was given to the update() method.
     *
     * Note this function will call the solve function internally.
     * If you need both coefficients and covariance matrix it is
     * more efficient to call solve directly.
     *
     * @result is a three vector <a,b,c> such that the plane is given
     * by z = a * x + b * y + c
     */
    Vector3 getCoeffs() const
    {
	return solve().getCoeffs();
    }

    /**
     * @brief calculate the covariance matrix
     *
     * Note this function will call the solve function internally.
     * If you need both coefficients and covariance matrix it is
     * more efficient to call solve directly.
     *
     * @result coefficient covariance matrix
     */
    Matrix3 getCovariance() const
    {
	return solve().getCovariance();
    }

    /**
     * Class to fit a plane which minimizes the squared distances of the points in normal direction.
     * For almost horizontal planes this is approximately the same as minimizing
     */
    class ResultNormal
    {
        Eigen::SelfAdjointEigenSolver<Matrix3> eig;
        Scalar offset;
    public:
        typedef Eigen::Hyperplane<Scalar, 3> Plane;
        ResultNormal(const PlaneFitting<Scalar>& sum )
        {
            Matrix3 moments;
            Vector3 mu(sum.x, sum.y, sum.z);
            moments << sum.xx, sum.xy, sum.xz,
                       sum.xy, sum.yy, sum.yz,
                       sum.xz, sum.yz, sum.zz;
            if(sum.n > 0.0)
            {
                moments -= mu * mu.transpose() * (1.0/sum.n);
                mu *= (1.0/sum.n);
            }
            eig.computeDirect(moments, Eigen::ComputeEigenvectors);
            offset = -eig.eigenvectors().col(0).dot(mu);
        }

        Vector3 getNormal() const
        {
            return eig.eigenvectors().col(0);
        }

        Scalar getOffset() const
        {
            return offset;
        }

        /**
         * Returns a plane parametrized by normal vector and offset
         */
        Plane getPlane() const
        {
            return Plane(getNormal(), getOffset());
        }

        /**
         * Returns the chi^2 error
         */
        Scalar getResiduals() const
        {
            return eig.eigenvalues()[0];
        }

    };

    ResultNormal solveNormal() const
    {
        return ResultNormal(*this);
    }
    Vector3 getNormal() const
    {
        return solveNormal().getNormal();
    }
};

}

#endif

