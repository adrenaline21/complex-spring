#include <Eigen/Core>
#include <Eigen/Sparse>
#include <complex>
#include <vector>
#include <iostream>

using Complexd = std::complex<double>;
using Vec3Cd = Eigen::Matrix<Complexd, 3, 1> ;
using T = Eigen::Triplet<double>;

int maxIte = 2;
double x0 = 1.5, l0 = 1.;
double dt = 0.5;
double ss2 = 0;

Complexd singleSpring(Complexd k) {
    Vec3Cd x(x0, 0, 0), v(0, 0, 0);
    std::cout << k << ' ' << x(0) << ' ';
    for (int ite = 0; ite < maxIte; ite++) {
        x += v * dt;
        Complexd norm = std::sqrt(x(0) * x(0));
        std::cout << norm << ' ' << x.norm() << std::endl;
        v += - k * (norm - l0) / norm * x * dt;
        std::cout << x(0) << ' ';
    }
    if (k == Complexd(2, 0))
        ss2 = real(x(0));
    std::cout << std::endl;
    return (x(0) - ss2) * (x(0) - ss2);
}

int main() {
    /*
    singleSpring(2);
    std::cout << singleSpring(2) << std::endl;
    std::cout << singleSpring(3.01) << ' ' << singleSpring(3) << ' ' << singleSpring(2.99) << std::endl;
    std::cout << singleSpring(3.01) - singleSpring(3) << std::endl;
    std::cout << singleSpring(Complexd(3, 0.01)) << std::endl;
    */
    std::vector<T> coefficients;
    coefficients.clear();
    coefficients.push_back(T(0, 0, 1));
    coefficients.push_back(T(0, 0, 1));
    coefficients.push_back(T(0, 0, 1));
    Eigen::SparseMatrix<double> mat(1, 1);
    mat.setFromTriplets(coefficients.begin(), coefficients.end());
    std::cout << mat.coeffRef(0, 0) << std::endl;
}