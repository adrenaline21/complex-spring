#ifndef __common_h__
#define __common_h__

#include <complex>
#include <vector>
#include <Eigen/Core>

template <class T_VAL> using Array = std::vector<T_VAL>;
using Vec3 = Eigen::Vector3d;
using Complexd = std::complex<double>;

#endif