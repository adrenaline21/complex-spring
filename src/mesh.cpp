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

void MeshR::Write(std::ofstream& fout) {
    //fout.precision(15);
    for (int i = 0; i < num_vertices; i++) {
        fout.write((char*)&vertices[i].x(0),sizeof(double));
        fout.write((char*)&vertices[i].x(1),sizeof(double));
        fout.write((char*)&vertices[i].x(2),sizeof(double));
        fout.write((char*)&vertices[i].n(0),sizeof(double));
        fout.write((char*)&vertices[i].n(1),sizeof(double));
        fout.write((char*)&vertices[i].n(2),sizeof(double));
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
double MeshT<T_VAL>::EvalStatic() {
    double retVal = 0;
    for (int i = 0; i < num_vertices; i++) {
        retVal += 0;//vertices[i].m * kG.dot(vertices[i].x);
    }
    return retVal;
}

void MeshR::GenerateLossFrame(double kStiffness) {
    Init();
    std::ofstream output(path + "/frames/0", std::ios::out);
    Write(output);
    output.close();
    for (int fr = 1; fr <= num_frames; fr++) {
        //ExplicitAdvance(kStiffness);
        ImplicitAdvance(kStiffness);
        std::ofstream frame_out(path + "/frames/" + std::to_string(fr), std::ios::out);
        Write(frame_out);
        if (fr == compared_frame) {
            std::ofstream loss_frame_out(path + "/loss_frame", std::ios::out|std::ios::binary);
            Write(loss_frame_out);
        }
    }
}

void MeshR::Read(std::ifstream& fin) {
    double buf[6];
    for (int i = 0; i < num_vertices; i++) {
        fin.read((char*)buf, sizeof(buf));
        vertices[i].x = Vec3T(buf[0], buf[1], buf[2]);
        vertices[i].n = Vec3T(buf[3], buf[4], buf[5]);
        //fin >> //vertices[i].index >> 
            //vertices[i].x(0) >> vertices[i].x(1) >> vertices[i].x(2) >>
            //vertices[i].n(0) >> vertices[i].n(1) >> vertices[i].n(2);
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