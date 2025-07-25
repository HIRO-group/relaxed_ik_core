#include "relaxed_ik.hpp"
#include <iostream>

int main() {
    RelaxedIKCpp solver;
    std::vector<double> pos{0.0,0.0,0.0};
    std::vector<double> quat{0.0,0.0,0.0,1.0};
    std::vector<double> tol{0.0,0.0,0.0,0.0,0.0,0.0};
    auto out = solver.solve_position(pos, quat, tol);
    std::cout << "Output size: " << out.size() << std::endl;
    return 0;
}
