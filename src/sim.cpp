#include <iostream>

#include "mesh.h"

const Vec3 kG(0, 0, -1);
const double kStiffness = 5, kAirdrag = 0.2;
double dt = 0.05;
std::string path;

Mesh mesh;

double evalLoss() {
    double retVal = 0;
    Vec3 a;
    for (int i = 0; i < mesh.num_vertices; i++) {
        retVal += (mesh.vertices[i].x - a).norm();
    }
    return retVal;
}

double evalStatic() {
    double retVal;
    for (int i = 0; i < mesh.num_vertices; i++) {
        retVal += mesh.vertices[i].m * mesh.vertices[i].v.norm() * mesh.vertices[i].v.norm() / 2.;
    }
    return retVal;
}

void explicitSolver(bool stop_when_static = true) {
    double lastStatik = 0;
    int fr = 1;
    for (; fr < 200; fr++) {
        for (auto& vertex : mesh.vertices) {
            vertex.x += vertex.v * dt;
            vertex.v += kG * dt;// - kAirdrag * vertex.v * dt;
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
        double statik = evalStatic();
        std::cout << fr << ':' << statik << "    ";
        //if (K < lastK)
            //break;
        lastStatik = statik;
        std::ofstream frame_out(path + "/main/" + std::to_string(fr), std::ios::out);
        mesh.Write(frame_out);
    }
}

int main(int argc, char *argv[]) {
    path = std::string(argv[1]);
    std::ifstream mesh_init_file(path + "/mesh_init", std::ios::in);
    mesh.Init(mesh_init_file);
    std::cout << mesh.num_frames << ' ' << mesh.num_vertices << std::endl;
    std::ofstream output(path + "/main/0", std::ios::out);
    mesh.Write(output);
    explicitSolver(false);
}