#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <iostream>
#include <Eigen/Geometry>

#include "mesh.h"

template<class T_VAL>
void MeshT<T_VAL>::ComputeNormal() {
    for (auto& vertex : vertices)
        vertex.n = Vec3T::Zero();
    for (int i = 0; i < num_faces; i++) {
        Vec3T
            x0 = vertices[indices[3 * i    ]].x,
            x1 = vertices[indices[3 * i + 1]].x,
            x2 = vertices[indices[3 * i + 2]].x;
        Vec3T n = (x1 - x0).cross(x2 - x1);
        vertices[indices[3 * i    ]].n += n,
        vertices[indices[3 * i + 1]].n += n,
        vertices[indices[3 * i + 2]].n += n;
    }
    for (auto& vertex : vertices)
        vertex.n.normalize();
}

template<class T_VAL>
void MeshT<T_VAL>::Write(std::ofstream& fout) {
    for (int i = 0; i < num_vertices; i++) {
        fout << vertices[i].x.transpose() << ' ' << vertices[i].n.transpose() << std::endl;
    }
    // The faces don't change for now.
    /*
    for (int i = 0; i < num_faces * 3; i++) {
        fout << indices[i] << ' ';
    }
    fout << std::endl;
    */
}

template<class T_VAL>
void MeshT<T_VAL>::ExplicitAdvance(double dt, double kStiffness) {
    for (auto& vertex : vertices) {
        vertex.x += vertex.v * dt;
        vertex.f = vertex.m * kG;
    }
    for (const auto& e : edges) {
        Eigen::Matrix<T_VAL, 3, 1> x01 = vertices[e.x1].x - vertices[e.x0].x;
        T_VAL len = x01.norm();
        len = std::sqrt(x01(0) * x01(0) + x01(1) * x01(1) + x01(2) * x01(2));
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
double MeshT<T_VAL>::EvalStatic() {
    double retVal = 0;
    for (int i = 0; i < num_vertices; i++) {
        retVal += vertices[i].m * kG.dot(vertices[i].x);
    }
    return retVal;
}

void MeshR::GenerateLossFrame(double kStiffness) {
    Init();
    std::ofstream output(path + "/frames/0", std::ios::out);
    Write(output);
    output.close();
    for (int fr = 1; fr <= num_frames; fr++) {
        ExplicitAdvance(dt, kStiffness);
        std::ofstream frame_out(path + "/frames/" + std::to_string(fr), std::ios::out);
        Write(frame_out);
        if (fr == compared_frame) {
            std::ofstream loss_frame_out(path + "/loss_frame", std::ios::out);
            Write(loss_frame_out);
        }
    }
}

void MeshR::Read(std::ifstream& fin) {
    for (int i = 0; i < num_vertices; i++) {
        fin >> //vertices[i].index >> 
            vertices[i].x(0) >> vertices[i].x(1) >> vertices[i].x(2) >>
            vertices[i].n(0) >> vertices[i].n(1) >> vertices[i].n(2);
    }
}

void MeshR::SetUpOpenGl() {
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STREAM_DRAW);  

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(uint), &indices[0], GL_STATIC_DRAW);

    // set the vertex attribute pointers
    // vertex Positions
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, x));
    // vertex normals
    glEnableVertexAttribArray(1);	
    glVertexAttribPointer(1, 3, GL_DOUBLE, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, n));
    glBindVertexArray(0);
}

void MeshR::Draw() {
    glBindVertexArray(VAO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STREAM_DRAW);  
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}