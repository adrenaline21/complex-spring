#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
using Vec3 = Eigen::Vector3d;

int N = 5;

int main(int argc, char *argv[]) {
    std::string path = std::string(argv[1]) + "/mesh_init";
    std::ofstream fout(path, std::ios::out);
    fout << 1000 << std::endl << N * N << std::endl;
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) {
            fout << i << ' ' << j << ' ';
            fout << i << ' ' << j << ' ' << double(rand())/double(RAND_MAX) << std::endl;
        }
    int num_faces = 2 * (N - 1) * (N - 1);
    fout << num_faces << std::endl;
    for (int i = 0; i < N - 1; i++)
        for (int j = 0; j < N - 1; j++) {
            int index = i * N + j;
            fout << index << ' ' << index + N << ' ' << index + N + 1 << std::endl;
            fout << index << ' ' << index + N + 1 << ' ' << index + 1 << std::endl;
        }
    fout.close();
}