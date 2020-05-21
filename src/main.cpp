#include <iostream>

#include "mesh.h"
#include "CFD.h"

std::string path;
MeshR meshR;
MeshC meshC;
MeshT<std::complex<double>> meshc;

int main(int argc, char *argv[]) {
    path = std::string(argv[1]);
    meshR.path = path; meshC.path = path;
    meshR.GenerateLossFrame(5);
    std::cout << meshR.EvalLossSameFrame(4) << std::endl;
    std::cout << (meshR.EvalLossSameFrame(4.01) - meshR.EvalLossSameFrame(3.99)) / 0.02 << std::endl;
    std::cout << std::imag(meshC.EvalLossSameFrame(std::complex<double>(4, 0.01))) / 0.01<< std::endl;
    
    double step = 1e-4, x = 4;
    double lx, lx_pos, lx_neg, grad, hess;
    int ite = 0;
    do {
        ite++;
        lx = meshR.EvalLossSameFrame(x);
        lx_pos = meshR.EvalLossSameFrame(x + step);
        lx_neg = meshR.EvalLossSameFrame(x - step);
        grad = (lx_pos - lx_neg) / (2. * step);
        hess = (lx_pos + lx_neg - 2 * lx) / (step * step);
        std::cout << "Ite." << ite << ' ' << x << ':' << lx << ' ' << lx_pos << ' ' << lx_neg << ' ' << grad << ' ' << hess << std::endl;
        std::cout << imag(meshC.EvalLossSameFrame(std::complex<double>(x, 0.01))) / 0.01 << std::endl;
        x -= lx / grad;
    } while (fabs(grad) > 1e-4 && ite < 10);
}
    