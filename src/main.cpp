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
    std::cout << meshR.EvalLossSameFrame(6) << std::endl;
    std::cout << meshR.EvalLossSameFrame(std::complex<double>(5, 0.1)) << std::endl;
    /*
    for (double kStiffness = 1; kStiffness < 10; kStiffness += 0.5) {
        double loss;
        if (kStiffness == 5)
            mesh.explicitSolver(kStiffness, loss, false, true);
        else
            mesh.explicitSolver(kStiffness, loss);
    }
    
    double step = 1e-4, x = 4;
    double lx, lx_pos, lx_neg, grad, hess;
    int ite = 0;
    do {
        ite++;
        mesh.explicitSolver(x, lx);
        mesh.explicitSolver(x + step, lx_pos);
        mesh.explicitSolver(x - step, lx_neg);
        grad = (lx_pos - lx_neg) / (2. * step);
        hess = (lx_pos + lx_neg - 2 * lx) / (step * step);
        std::cout << x << ':' << lx << ' ' << lx_pos << ' ' << lx_neg << ' ' << grad << ' ' << hess << std::endl;
        std::complex<double> loss_c;
        meshc.explicitSolver(std::complex<double>(x, 0.1), loss_c);
        std::cout << imag(loss_c) / 0.1 << std::endl;
        x -= lx / grad;
    } while (fabs(grad) > 1e-4 && ite < 10);
    for (int i = 0; i < 10; i++) {
        std::ifstream mesh_init_file(path + "/mesh_init", std::ios::in);
        mesh.Init(mesh_init_file);
        mesh_init_file.close();
        std::ofstream output(path + "/main/0", std::ios::out);
        mesh.Write(output);
        kStiffness = 4 + pow(0.1, i);
        explicitSolver();
        std::cout << (loss - base) / (kStiffness - 4) << std::endl;
    }
   
    std::complex<double> loss;
    double loss1, loss2;
    meshc.explicitSolver(std::complex<double>(4, 0.1), loss);
    meshc.explicitSolver(std::complex<double>(4, 0.0001), loss);
    meshc.explicitSolver(std::complex<double>(2, 0.1), loss);
    meshc.explicitSolver(std::complex<double>(2, 0.0001), loss);
    double step2 = 1e-1;
    mesh.explicitSolver(4 + step, loss1);
    mesh.explicitSolver(4 - step, loss2);
    std::cout << (loss1 - loss2) / (2 * step2) << std::endl;
    */
}