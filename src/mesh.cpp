#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <iostream>
#include <Eigen/Geometry>

#include "mesh.h"

void Mesh::Init(std::ifstream& fin) {
    double mass = 1.;
    fin >> num_frames >> num_vertices;
    vertices.resize(num_vertices);
    for (int i = 0; i < num_vertices; i++) {
        Vertex p;
        p.index = i;
        p.m = mass;
        fin >> p.coord(0) >> p.coord(1);
        fin >> p.x(0) >> p.x(1) >> p.x(2);
        p.v = Vec3::Zero();
        vertices[i] = p;
    }
    fin >> num_faces;
    indices.resize(num_faces * 3);
    for (int i = 0; i < num_faces * 3; i++) {
        fin >> indices[i];
    }
    num_edges = 0;
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
    std::cout << "Edges:" << num_edges << std::endl;
    ComputeNormal();
}

void Mesh::Read(std::ifstream& fin) {
    for (int i = 0; i < num_vertices; i++) {
        fin >> //vertices[i].index >> 
            vertices[i].x(0) >> vertices[i].x(1) >> vertices[i].x(2) >>
            vertices[i].n(0) >> vertices[i].n(1) >> vertices[i].n(2);
    }
}

void Mesh::Write(std::ofstream& fout) {
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

void Mesh::ComputeNormal() {
    for (auto& vertex : vertices)
        vertex.n = Vec3::Zero();
    for (int i = 0; i < num_faces; i++) {
        Vec3 
            x0 = vertices[indices[3 * i    ]].x,
            x1 = vertices[indices[3 * i + 1]].x,
            x2 = vertices[indices[3 * i + 2]].x;
        Vec3 n = (x1 - x0).cross(x2 - x1);
        vertices[indices[3 * i    ]].n += n,
        vertices[indices[3 * i + 1]].n += n,
        vertices[indices[3 * i + 2]].n += n;
    }
    for (auto& vertex : vertices)
        vertex.n.normalize();
}

void Mesh::SetUpOpenGl() {
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
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, x));
    // vertex normals
    glEnableVertexAttribArray(1);	
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, n));
    glBindVertexArray(0);
}

void Mesh::Draw() {
    glBindVertexArray(VAO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STREAM_DRAW);  
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}