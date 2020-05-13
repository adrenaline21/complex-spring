#include <iostream>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <complex>

#include "mesh.h"

const Vec3 kG(0, 0, -1);
const double kStiffness = 3, kAirdrag = 0.2;
double dt = 0.05;
int N;

Mesh mesh;

double evalObjective(const Eigen::VectorXd& X, const Eigen::VectorXd& y) {
    double retVal = 0;
    Array<Vec3> x;
    x.resize(N);
    for (int i = 0; i < N; i++) {
        x[i](0) = X(3 * i);
        x[i](1) = X(3 * i + 1);
        x[i](2) = X(3 * i + 2);
    }
    for (const auto& e : mesh.edges) {
        Vec3 x01 = x[e.x1] - x[e.x0];
        double len = x01.norm();
        retVal += kStiffness/2. * (len - e.len) * (len - e.len);
    }
    Eigen::VectorXd xy = X - y;
    for (int i = 0; i < N; i++) {
        double k = .5 / (dt * dt) * mesh.vertices[i].m;
        for (int j = 0; j < 3; j++) {
            double v = xy(3 * i + j);
            retVal += k * v * v;
        }
    }
    return retVal;
}

void evalGradient(Eigen::VectorXd& grad, const Eigen::VectorXd& X, const Eigen::VectorXd& y) {
    Array<Vec3> f, x;
    // Grad E(x)
    f.resize(N);    x.resize(N);
    for (int i = 0; i < N; i++) {
        f[i].setZero();
        x[i](0) = X(3 * i);
        x[i](1) = X(3 * i + 1);
        x[i](2) = X(3 * i + 2);
    }
    for (const auto& e : mesh.edges) {
        Vec3 x01 = x[e.x1] - x[e.x0];
        double len = x01.norm();
        f[e.x0] -= 1./mesh.vertices[e.x0].m * kStiffness * (len - e.len) * x01 * dt;
        f[e.x1] += 1./mesh.vertices[e.x1].m * kStiffness * (len - e.len) * x01 * dt; 
    }
    // Adding to x - y
    Eigen::VectorXd xy = X - y;
    for (int i = 0; i < N; i++) {
        double c = mesh.vertices[i].m / (dt * dt);
        grad(3 * i    ) = f[i](0) + c * xy(3 * i);
        grad(3 * i + 1) = f[i](1) + c * xy(3 * i + 1);
        grad(3 * i + 2) = f[i](2) + c * xy(3 * i + 2);
    }
    //std::cout << grad.transpose() << std::endl;
}

void evalHessian(Eigen::SparseMatrix<double>& hessian) {
    hessian.resize(3 * N, 3 * N);
}

void Descent(Eigen::VectorXd& x) {
    Eigen::VectorXd y;
    y.resize(3 * N);
    Array<Vec3> f;
    f.resize(N);
    for (int i = 0; i < N; i++) {
        f[i] = mesh.vertices[i].x + dt * mesh.vertices[i].v + dt * dt * kG;
    }
    for (int i = 0; i < N; i++) {
        y(3 * i    ) = f[i](0);
        y(3 * i + 1) = f[i](1);
        y(3 * i + 2) = f[i](2);
    }
    x = y;
    Eigen::VectorXd grad;
    grad.resize(3 * N);
    grad.setZero();
    int ite = 0;
    double kB = 0.5, kY = 0.03;
    evalGradient(grad, x, y);
    std::cout << "OBJ:" <<  - evalObjective(x, y) + evalObjective(x - 0.00001 * grad, y) << ' ' << -0.00001 * grad.dot(grad) << std::endl;
    for (; ite < 10; ite++) {
        double obj = evalObjective(x, y);
        //std::cout << obj << ' ';
        evalGradient(grad, x, y);
        double norm = grad.norm();
        std::cout << norm << ' ';
        if (norm < 1e-2)
            break;
        double kA = 1./kB; Eigen::VectorXd xTemp = x;
        do {
            kA *= kB;
            xTemp = x - kA * grad;
            //std::cout << evalObjective(xTemp, y) << ' ';
        } while (evalObjective(xTemp, y) > obj - kY * kA * norm * norm);
        //std::cout << std::endl;
        x = xTemp;
    }
    std::cout << std::endl;
}

int main(int argc, char *argv[]) {
    std::string path = std::string(argv[1]);
    std::ifstream mesh_file(path + "/mesh", std::ios::in);
    mesh.Init(mesh_file);
    std::cout << mesh.num_frames << ' ' << mesh.num_vertices << std::endl;
    N = mesh.num_vertices;
    std::ofstream output(path + "/0", std::ios::out);
    mesh.Write(output);
    Eigen::VectorXd x;
    x.resize(3 * N);
    for (int fr = 1; fr < mesh.num_frames; fr++) {
        Descent(x);
        for (int i = 0; i < N; i++)
            if (i != 0 && i != 4) {
                Vec3 newx;
                newx(0) = x(3 * i);
                newx(1) = x(3 * i + 1);
                newx(2) = x(3 * i + 2);
                mesh.vertices[i].v = (newx - mesh.vertices[i].x) / dt;
                mesh.vertices[i].x = newx;
            }
        mesh.ComputeNormal();
        std::ofstream frame_out(path + std::to_string(fr), std::ios::out);
        mesh.Write(frame_out);
    }
}