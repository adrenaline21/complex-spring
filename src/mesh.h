#ifndef __mesh_h__
#define __mesh_h__

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <fstream>

#include "common.h"

struct Vertex {
    int index;
    double m;
    Eigen::Vector2i coord;
    Vec3 x, v, n;
};

struct Edge {
    int x0, x1;
    double len;
};
/*
struct GLVertex {
    glm::vec3 x;
    glm::vec3 n;
};
*/

struct Mesh {
    int num_vertices, num_faces, num_frames, num_edges;
    Array<Vertex> vertices;
    Array<Edge> edges;
    //Array<GLVertex> GLvertices;

    Array<uint> indices;
    //Array<Eigen::Vector3i> faces; //right-hand normal sequence
    
    void Init(std::ifstream&);
    void Read(std::ifstream&);
    void Write(std::ofstream&);
    void ComputeNormal();

    uint VAO, VBO, EBO;
    void SetUpOpenGl();
    void Draw();
};

#endif