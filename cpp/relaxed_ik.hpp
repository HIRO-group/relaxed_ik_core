#ifndef RELAXED_IK_CPP_HPP
#define RELAXED_IK_CPP_HPP

#include <cstdint>
#include <vector>
#include <string>

extern "C" {
    typedef struct Opt {
        const double* data;
        int32_t length;
    } Opt;

    struct RelaxedIK;

    RelaxedIK* relaxed_ik_new(const char* path_to_setting);
    void relaxed_ik_free(RelaxedIK* ptr);
    void reset(RelaxedIK* ptr, const double* joint_state, int32_t joint_state_length);
    Opt solve_position(RelaxedIK* ptr, const double* pos_goals, int32_t pos_length,
                       const double* quat_goals, int32_t quat_length,
                       const double* tolerance, int32_t tolerance_length);
    Opt solve_velocity(RelaxedIK* ptr, const double* pos_vels, int32_t pos_length,
                       const double* rot_vels, int32_t rot_length,
                       const double* tolerance, int32_t tolerance_length);
    Opt get_ee_positions(RelaxedIK* ptr);
}

class RelaxedIKCpp {
public:
    explicit RelaxedIKCpp(const std::string& path = "") {
        const char* cpath = path.empty() ? nullptr : path.c_str();
        inner = relaxed_ik_new(cpath);
    }
    ~RelaxedIKCpp() {
        relaxed_ik_free(inner);
    }

    void reset(const std::vector<double>& joint_state) {
        reset(inner, joint_state.data(), static_cast<int32_t>(joint_state.size()));
    }

    std::vector<double> solve_position(const std::vector<double>& positions,
                                       const std::vector<double>& orientations,
                                       const std::vector<double>& tolerances) {
        Opt opt = ::solve_position(inner,
                                   positions.data(), static_cast<int32_t>(positions.size()),
                                   orientations.data(), static_cast<int32_t>(orientations.size()),
                                   tolerances.data(), static_cast<int32_t>(tolerances.size()));
        std::vector<double> out(opt.data, opt.data + opt.length);
        return out;
    }

    std::vector<double> solve_velocity(const std::vector<double>& linear,
                                       const std::vector<double>& angular,
                                       const std::vector<double>& tolerances) {
        Opt opt = ::solve_velocity(inner,
                                   linear.data(), static_cast<int32_t>(linear.size()),
                                   angular.data(), static_cast<int32_t>(angular.size()),
                                   tolerances.data(), static_cast<int32_t>(tolerances.size()));
        std::vector<double> out(opt.data, opt.data + opt.length);
        return out;
    }

    std::vector<double> get_end_effector_positions() {
        Opt opt = ::get_ee_positions(inner);
        std::vector<double> out(opt.data, opt.data + opt.length);
        return out;
    }

private:
    RelaxedIK* inner {nullptr};
};

#endif // RELAXED_IK_CPP_HPP
