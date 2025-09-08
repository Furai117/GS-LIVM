#include <cassert>
#include <cmath>
#include <iostream>

struct Quaternion {
    float x, y, z, w;
};

Quaternion rotmat2qvec(const float R[3][3]) {
    Quaternion q{};
    float trace = R[0][0] + R[1][1] + R[2][2];
    if (trace > 0.f) {
        float s = std::sqrt(trace + 1.f) * 2.f;
        q.w = 0.25f * s;
        q.x = (R[2][1] - R[1][2]) / s;
        q.y = (R[0][2] - R[2][0]) / s;
        q.z = (R[1][0] - R[0][1]) / s;
    } else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
        float s = std::sqrt(1.f + R[0][0] - R[1][1] - R[2][2]) * 2.f;
        q.w = (R[2][1] - R[1][2]) / s;
        q.x = 0.25f * s;
        q.y = (R[0][1] + R[1][0]) / s;
        q.z = (R[0][2] + R[2][0]) / s;
    } else if (R[1][1] > R[2][2]) {
        float s = std::sqrt(1.f + R[1][1] - R[0][0] - R[2][2]) * 2.f;
        q.w = (R[0][2] - R[2][0]) / s;
        q.x = (R[0][1] + R[1][0]) / s;
        q.y = 0.25f * s;
        q.z = (R[1][2] + R[2][1]) / s;
    } else {
        float s = std::sqrt(1.f + R[2][2] - R[0][0] - R[1][1]) * 2.f;
        q.w = (R[1][0] - R[0][1]) / s;
        q.x = (R[0][2] + R[2][0]) / s;
        q.y = (R[1][2] + R[2][1]) / s;
        q.z = 0.25f * s;
    }
    if (q.w < 0.f) {
        q.x = -q.x;
        q.y = -q.y;
        q.z = -q.z;
        q.w = -q.w;
    }
    return q;
}

void qvec2rotmat(const Quaternion& q, float R[3][3]) {
    float x = q.x, y = q.y, z = q.z, w = q.w;
    R[0][0] = 1.f - 2.f * y * y - 2.f * z * z;
    R[0][1] = 2.f * x * y - 2.f * z * w;
    R[0][2] = 2.f * x * z + 2.f * y * w;
    R[1][0] = 2.f * x * y + 2.f * z * w;
    R[1][1] = 1.f - 2.f * x * x - 2.f * z * z;
    R[1][2] = 2.f * y * z - 2.f * x * w;
    R[2][0] = 2.f * x * z - 2.f * y * w;
    R[2][1] = 2.f * y * z + 2.f * x * w;
    R[2][2] = 1.f - 2.f * x * x - 2.f * y * y;
}

int main() {
    const float R[3][3] = {{1.f, 0.f, 0.f}, {0.f, 0.f, -1.f}, {0.f, 1.f, 0.f}}; // 90 deg around X
    Quaternion q = rotmat2qvec(R);
    float R2[3][3];
    qvec2rotmat(q, R2);

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            assert(std::abs(R[i][j] - R2[i][j]) < 1e-5f);
        }
    }

    const float sqrt_half = std::sqrt(0.5f);
    assert(std::abs(q.x - sqrt_half) < 1e-5f);
    assert(std::abs(q.y) < 1e-5f);
    assert(std::abs(q.z) < 1e-5f);
    assert(std::abs(q.w - sqrt_half) < 1e-5f);

    std::cout << "Quaternion coefficients: " << q.x << " " << q.y << " " << q.z << " " << q.w << std::endl;
    return 0;
}
