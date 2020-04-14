#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
using Vec3 = Eigen::Vector3d;

int N = 100;

int main() {
    int vertexsize = 2 * (N - 1) * (N - 1), last_frame = 100;
    double h[N * N];
    int tri[vertexsize][3];
    for (int i = 0; i < vertexsize / 2; i++) {
        int x = i / (N - 1), y = i % (N - 1);
        tri[i][0] = x * N + y;
        tri[i][1] = x * N + y + N;
        tri[i][2] = x * N + y + N + 1;
    }
    for (int i = vertexsize / 2; i < vertexsize; i++) {
        int x = (i - vertexsize / 2) / (N - 1), y = (i - vertexsize / 2) % (N - 1);
        tri[i][0] = x * N + y;
        tri[i][1] = x * N + y + 1;
        tri[i][2] = x * N + y + N + 1;
    }

    double dx = 2. * M_PI / double(N - 1.), dt = 2. * M_PI / double(last_frame + 1.);
    std::cout << dx << ' ' << dt << std::endl;

    std::ofstream main_file("data/tritest/main", std::ios::out);
    main_file << vertexsize * 3 << ' ' << last_frame;
    main_file.close();

    Vec3 v[N * N];
    for (int t = 0; t <= last_frame; t++) {
        std::cout << t << ' ';
        fflush(stdout);
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++) {
                v[i * N + j] = Vec3(i, j, sin(dt * t) * sin(i * dx) * sin(j * dx) * N / 2.);
            }
        std::cout << t << ' ';
        fflush(stdout);
        std::ofstream frame_file("data/tritest/" + std::to_string(t), std::ios::out);
        for (int i = 0; i < vertexsize; i++) {
            Vec3 x0 = v[tri[i][0]], x1 = v[tri[i][1]], x2 = v[tri[i][2]];
            Vec3 norm = (x2-x0).cross(x1-x0).normalized();
            if (norm.z() < 0)
                norm = -norm;
            frame_file << x0.transpose() << ' ' << norm.transpose() << std::endl;
            frame_file << x1.transpose() << ' ' << norm.transpose() << std::endl;
            frame_file << x2.transpose() << ' ' << norm.transpose() << std::endl;
        }
        frame_file.close();
    }
}