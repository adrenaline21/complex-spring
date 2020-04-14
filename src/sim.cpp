#include <iostream>

#include "mesh.h"

const Vec3 kG(0, 0, -1);
const double kStiffness = 5, kAirdrag = 0.2;
double dt = 0.05;

Mesh mesh;

int main(int argc, char *argv[]) {
    std::string path = std::string(argv[1]);
    std::ifstream mesh_file(path + "/mesh", std::ios::in);
    mesh.Init(mesh_file);
    std::cout << mesh.num_frames << ' ' << mesh.num_vertices << std::endl;
    std::ofstream output(path + "/0", std::ios::out);
    mesh.Write(output);
    for (int fr = 1; fr < mesh.num_frames; fr++) {
        for (auto& vertex : mesh.vertices) {
            vertex.x += vertex.v * dt;
            vertex.v += kG * dt - kAirdrag * vertex.v * dt;
        }
        for (const auto& e : mesh.edges) {
            Vec3 x01 = mesh.vertices[e.x1].x - mesh.vertices[e.x0].x;
            double len = x01.norm();
            mesh.vertices[e.x0].v += 1./mesh.vertices[e.x0].m * kStiffness * (len - e.len) * x01 * dt; 
            mesh.vertices[e.x1].v -= 1./mesh.vertices[e.x1].m * kStiffness * (len - e.len) * x01 * dt; 
        }
        mesh.vertices[0].v = Vec3::Zero(); 
        mesh.vertices[4].v = Vec3::Zero();
        //mesh.vertices[20].v = Vec3::Zero(); 
        //mesh.vertices[24].v = Vec3::Zero();
        mesh.ComputeNormal();
        std::ofstream frame_out(path + std::to_string(fr), std::ios::out);
        mesh.Write(frame_out);
    }
}