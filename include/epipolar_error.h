#ifndef CERES_STEREO_EPIPOLARERROR_H_
#define CERES_STEREO_EPIPOLARERROR_H_

#include "ceres/ceres.h"
#include "so3.hpp"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;


using Eigen::Matrix;
using Eigen::Vector2d;
using Sophus::SO3Group;

namespace ceres{
namespace epipolar {
struct EpipolarError {
	EpipolarError(Vector3d observerd_p1, Vector3d observerd_p2) 
		: p1_(observerd_p1), p2_(observerd_p2) {}

	bool operator()(const double* const omega, const double* const tt, double* residual) const {
		typedef SO3Group<double> SO3Type;
		typedef SO3Group<double>::Tangent Tangent;
		typedef SO3Group<double>::Transformation Transformation;
		SO3Type R = SO3Type::exp(SO3Type::Tangent(omega[0], omega[1], omega[2]));
		Tangent t(tt[0], tt[1], tt[2]);
		t.normalize();
		Transformation t_hat = SO3Type::hat(t);
		residual[0] = p2_.transpose() * t_hat * R.matrix() * p1_;
		return true;
	}

	static ceres::CostFunction* Create(const Vector3d observerd_p1, const Vector3d observerd_p2) {
		return (new ceres::NumericDiffCostFunction<EpipolarError, ceres::CENTRAL, 1, 3, 3>(
					new EpipolarError( observerd_p1,  observerd_p2)));
	}
	
private:	
	Vector3d p1_;
	Vector3d p2_;
};
	
	
} //namespace epipolar
} //namespace ceres


#endif
