#ifndef __mesh_h__
#define __mesh_h__

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <fstream>
#include <iostream>
#include <complex>
#include <string>

#include "common.h"

template<class T_VAL = double>
struct VertexT {
    int index;
    T_VAL m;
    Eigen::Vector2i coord;
    Eigen::Matrix<T_VAL, 3, 1> x, v, n, f;
};

//typedef VertexT<std::complex<double>> VertexCd;

struct Edge {
    int x0, x1;
    double len;
};

template<class T_VAL>
struct MeshT {
    using Vec3T = Eigen::Matrix<T_VAL, 3, 1>;
    using VecXT = Eigen::Matrix<T_VAL, Eigen::Dynamic, 1>;
    std::string path;
    Vec3T kG = Vec3T(0, 0, -1);
    double dt = 0.05;

    int num_vertices, num_faces, num_frames, num_edges;
    Array<VertexT<T_VAL>> vertices;
    Array<Edge> edges;
    Array<uint> indices;
    
    void Init();
    void ComputeNormal();
    void Write(std::ofstream&);

    int compared_frame = 100;
    void ExplicitAdvance(T_VAL kStiffness);

    void EvalGradient(T_VAL kStiffness, VecXT& grad, const VecXT& X, const VecXT& y);
    double EvalObjective(T_VAL kStiffness, const VecXT& X, const VecXT& y);
    void Descent(T_VAL kStiffness, VecXT&);
    void ImplicitAdvance(T_VAL kStiffness);

    T_VAL EvalLossSameFrame(T_VAL kStiffness);
    double EvalStatic();
};

template struct MeshT<std::complex<double>>;
using MeshC = MeshT<std::complex<double>>;

struct MeshR : MeshT<double> {
    using Vertex = VertexT<double>;

    void Read(std::ifstream&);
    void GenerateLossFrame(double kStiffness);

    uint VAO, VBO, EBO;
    void SetUpOpenGl();
    void Draw();
};

template<class T_VAL>
void MeshT<T_VAL>::Init() {
    std::ifstream mesh_init_file(path + "/mesh_init", std::ios::in);
    double mass = 1.;
    mesh_init_file >> num_frames >> num_vertices;
    vertices.resize(num_vertices);
    for (int i = 0; i < num_vertices; i++) {
        VertexT<T_VAL> p;
        p.index = i;
        p.m = mass;
        mesh_init_file >> p.coord(0) >> p.coord(1);
        mesh_init_file >> p.x(0) >> p.x(1) >> p.x(2);
        p.v = Vec3T::Zero();
        p.f = Vec3T::Zero();
        vertices[i] = p;
    }
    mesh_init_file >> num_faces;
    indices.resize(num_faces * 3);
    for (int i = 0; i < num_faces * 3; i++) {
        mesh_init_file >> indices[i];
    }
    mesh_init_file.close();

    num_edges = 0;
    edges.clear();
    for (int i = 0; i < num_faces; i++) {
        int x[3];
        for (int j = 0; j < 3; j++)
            x[j] = indices[3 * i + j];
        for (int j = 0; j < 3; j++) {
            int x0 = fmin(x[j], x[(j+1)%3]), x1 = fmax(x[j], x[(j+1)%3]);
            bool found = false;
            for (int k = 0; k < num_edges; k++)
                if (x0 == edges[k].x0 && x1 == edges[k].x1) {
                    found = true;
                    break;
                }
            if (!found) {
                Edge e;
                e.x0 = x0; e.x1 = x1; e.len = (vertices[x0].x - vertices[x1].x).norm();
                edges.push_back(e);
                num_edges++;
            }
        }
    }
    //std::cout << "Edges:" << num_edges << std::endl;

    ComputeNormal();
}

template<class T_VAL>
void MeshT<T_VAL>::ExplicitAdvance(T_VAL kStiffness) {
    for (auto& vertex : vertices) {
        vertex.x += vertex.v * dt;
        vertex.f = vertex.m * kG;
    }
    for (const auto& e : edges) {
        Eigen::Matrix<T_VAL, 3, 1> x01 = vertices[e.x1].x - vertices[e.x0].x;
        T_VAL len = std::sqrt(x01(0) * x01(0) + x01(1) * x01(1) + x01(2) * x01(2));
        vertices[e.x0].f += kStiffness * (len - e.len) * x01 / len; 
        vertices[e.x1].f -= kStiffness * (len - e.len) * x01 / len; 
    }
    for (auto& vertex : vertices) {
        vertex.v += vertex.f / vertex.m * dt;
    }
    vertices[0].v = Vec3T::Zero(); 
    vertices[4].v = Vec3T::Zero();
    ComputeNormal();
}

template<class T_VAL>
T_VAL MeshT<T_VAL>::EvalLossSameFrame(T_VAL kStiffness) {
    T_VAL retVal = 0;
    Init();
    for (int fr = 1; fr <= compared_frame; fr++)
        ExplicitAdvance(kStiffness);
    std::ofstream test_eval(path + "/test_eval", std::ios::out);
    Write(test_eval);
    std::ifstream loss_frame_file(path + "/loss_frame", std::ios::in);
    double x, y, z, tmp;
    for (int i = 0; i < num_vertices; i++) {
        loss_frame_file >> x >> y >> z >> tmp >> tmp >> tmp;
        retVal += 
            (vertices[i].x(0) - x) * (vertices[i].x(0) - x) +
            (vertices[i].x(1) - y) * (vertices[i].x(1) - y) +
            (vertices[i].x(2) - z) * (vertices[i].x(2) - z);
    }
    loss_frame_file.close();
    return retVal;
}


template<class T_VAL>
void MeshT<T_VAL>::EvalGradient(T_VAL kStiffness, VecXT& grad, const VecXT& X, const VecXT& y) {
    int N = num_vertices;
    Array<Vec3T> f, x;
    // Grad E(x)
    f.resize(N);    x.resize(N);
    for (int i = 0; i < N; i++) {
        f[i].setZero();
        x[i](0) = X(3 * i);
        x[i](1) = X(3 * i + 1);
        x[i](2) = X(3 * i + 2);
    }
    for (const auto& e : edges) {
        Vec3T x01 = x[e.x1] - x[e.x0];
        T_VAL len = std::sqrt(x01(0) * x01(0) + x01(1) * x01(1) + x01(2) * x01(2));
        f[e.x0] -= kStiffness * (len - e.len) * x01 / len * dt; 
        f[e.x1] += kStiffness * (len - e.len) * x01 / len * dt;
    }
    // Adding to x - y
    VecXT xy = X - y;
    for (int i = 0; i < N; i++) {
        T_VAL c = vertices[i].m / (dt * dt);
        grad(3 * i    ) = f[i](0) + c * xy(3 * i);
        grad(3 * i + 1) = f[i](1) + c * xy(3 * i + 1);
        grad(3 * i + 2) = f[i](2) + c * xy(3 * i + 2);
    }
    //std::cout << grad.transpose() << std::endl;
}

template<class T_VAL>
double MeshT<T_VAL>::EvalObjective(T_VAL kStiffness, const VecXT& X, const VecXT& y) {
    int N = num_vertices;
    T_VAL retVal = 0;
    Array<Vec3T> x;
    x.resize(N);
    for (int i = 0; i < N; i++) {
        x[i](0) = X(3 * i);
        x[i](1) = X(3 * i + 1);
        x[i](2) = X(3 * i + 2);
    }
    for (const auto& e : edges) {
        Vec3T x01 = x[e.x1] - x[e.x0];
        T_VAL len = std::sqrt(x01(0) * x01(0) + x01(1) * x01(1) + x01(2) * x01(2));
        retVal += kStiffness/2. * (len - e.len) * (len - e.len);
    }
    VecXT xy = X - y;
    for (int i = 0; i < N; i++) {
        T_VAL k = .5 / (dt * dt) * vertices[i].m;
        for (int j = 0; j < 3; j++) {
            T_VAL v = xy(3 * i + j);
            retVal += k * v * v;
        }
    }
    return std::real(retVal);
}

template<class T_VAL>
void MeshT<T_VAL>::Descent(T_VAL kStiffness, VecXT& x) {
    int N = num_vertices;
    VecXT y;
    y.resize(3 * N);
    Array<Vec3T> f;
    f.resize(N);
    for (int i = 0; i < N; i++) {
        f[i] = vertices[i].x + dt * vertices[i].v + dt * dt * kG;
    } 
    for (int i = 0; i < N; i++) {
        y(3 * i    ) = f[i](0);
        y(3 * i + 1) = f[i](1);
        y(3 * i + 2) = f[i](2);
    }
    x = y;
    VecXT grad;
    grad.resize(3 * N);
    grad.setZero();
    EvalGradient(kStiffness, grad, x, y);
    double obj = EvalObjective(kStiffness, x, y);
    std::cout << grad.transpose() << std::endl;
    for (int i = 0; i < 3 * N; i++) {
        VecXT newx = x; newx(i) = x(i) + 1e-4
        std::cout << EvalObjective(kStiffness, newx, y) - obj << ' ';
    }
    std::cout << std::endl;
    
    int ite = 0;
    double kB = 0.5, kY = 0.03;
    //VecXT new_grad; new_grad.resize(3 * N);
    for (; ite < 10; ite++) {
        double obj = EvalObjective(kStiffness, x, y);
        EvalGradient(kStiffness, grad, x, y);
        double norm = grad.norm();
        if (norm < 1e-2)
            break;
        double kA = 1./kB; VecXT xTemp = x;
        do {
            kA *= kB;
            xTemp = x - kA * grad;
            //EvalGradient(kStiffness, new_grad, xTemp, y);
            //std::cout << evalObjective(xTemp, y) << ' ';
        } while (EvalObjective(kStiffness, xTemp, y) > obj - kY * kA * norm * norm);
        std::cout << std::endl;
        x = xTemp;
    }
}

template<class T_VAL>
void MeshT<T_VAL>::ImplicitAdvance(T_VAL kStiffness) {
    int N = num_vertices;
    VecXT x;
    Descent(kStiffness, x);
    for (int i = 0; i < N; i++)
        if (i != 0 && i != 4) {
            Vec3T newx;
            newx(0) = x(3 * i);
            newx(1) = x(3 * i + 1);
            newx(2) = x(3 * i + 2);
            vertices[i].v = (newx - vertices[i].x) / dt;
            vertices[i].x = newx;
        }
    ComputeNormal();
}

#endif