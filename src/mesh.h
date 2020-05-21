#ifndef __mesh_h__
#define __mesh_h__

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <fstream>
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
    void ExplicitAdvance(double dt, T_VAL kStiffness);

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
T_VAL MeshT<T_VAL>::EvalLossSameFrame(T_VAL kStiffness) {
    T_VAL retVal = 0;
    Init();
    for (int fr = 1; fr <= compared_frame; fr++)
        ExplicitAdvance(dt, kStiffness);
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

#endif