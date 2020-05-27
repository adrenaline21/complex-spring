#include <iostream>
#include <Eigen/Core>

#include "mesh.h"

std::string path;
MeshR meshR;
MeshC meshC;
MeshT<Complexd> meshc;
const Complexd Z4(std::sqrt(.5), std::sqrt(.5));

int main(int argc, char *argv[]) {
    Eigen::setNbThreads(8);
    std::cout.precision(15);

    path = std::string(argv[1]);
    meshR.path = path; meshC.path = path;
    meshR.GenerateLossFrame(5);
    
    /*
    double base = meshR.EvalLossSameFrame(2.);
    for (int i = 1; i <= 20; i++) {
        double step = pow(0.1, i);
        std::cout << step << std::fixed << std::endl;
        double basep = meshR.EvalLossSameFrame(2. + step), basen = meshR.EvalLossSameFrame(2. - step);
        std::cout << (basep - base) / step << std::endl;
        std::cout << (basep - basen) / (2. * step) << std::endl;
        std::cout << std::imag(meshC.EvalLossSameFrame(Complexd(2., step))) / step << std::endl;
    }
    */

    /*
    double step = 1e-1, x = 2;
    double lx, lx_pos, lx_neg, grad, hess;
    int ite = 0;
    do {
        //std::cout << "Ite." << ite << ':' << x << ' ';
        std::cout << x;
        ite++;
        lx = meshR.EvalLossSameFrame(x);
        lx_pos = meshR.EvalLossSameFrame(x + step);
        lx_neg = meshR.EvalLossSameFrame(x - step);
        //grad = (lx_pos - lx_neg) / (2. * step);
        grad = std::imag(meshC.EvalLossSameFrame(Complexd(x, step))) / step;
        //hess = (lx_pos + lx_neg - 2 * lx) / (step * step);
        hess = std::imag(meshC.EvalLossSameFrame(x + step * Z4) + meshC.EvalLossSameFrame(x - step * Z4)) / (step * step);
        x -= grad / hess;
        //std::cout << grad << ' ' << hess;
        std::cout << std::endl;
    } while (ite < 20);
    */

    /*
    for (double x = 0; x <= 10; x += .1) {
        std::cout << meshR.EvalLossSameFrame(x) << std::endl;
    }
    */
}
    