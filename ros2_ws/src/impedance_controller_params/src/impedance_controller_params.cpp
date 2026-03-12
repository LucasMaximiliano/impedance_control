#include <stdexcept>
#include "impedance_controller_params.hpp"
#include "toml.hpp"

namespace impedance_controller_params {
    params_t::params_t(const std::string& toml_path) {
        toml::value config;
        try {
            config = toml::parse(toml_path);
        } catch (const std::exception& e) {
            throw std::runtime_error("Failed to parse TOML file: " + std::string(e.what()));
        }

        try {
            auto inertia_data = toml::find<std::vector<std::vector<double>>>(config, "virtual_inertia_matrix");
            virtual_inertia_matrix_ << inertia_data[0][0], inertia_data[0][1],
                                       inertia_data[1][0], inertia_data[1][1];
            validate_mat(virtual_inertia_matrix_, "virtual_inertia_matrix");
            
            auto damping_data = toml::find<std::vector<std::vector<double>>>(config, "virtual_damping_matrix");
            virtual_damping_matrix_ << damping_data[0][0], damping_data[0][1],
                                       damping_data[1][0], damping_data[1][1];
            validate_mat(virtual_damping_matrix_, "virtual_damping_matrix");
            
            auto stiffness_data = toml::find<std::vector<std::vector<double>>>(config, "virtual_stiffness_matrix");
            virtual_stiffness_matrix_ << stiffness_data[0][0], stiffness_data[0][1],
                                         stiffness_data[1][0], stiffness_data[1][1];
            validate_mat(virtual_stiffness_matrix_, "virtual_stiffness_matrix");

            force_feedback_gain_ = toml::find<double>(config, "force_feedback_gain");
            validate_range(force_feedback_gain_, 0.0, MAX_FORCE_FEEDBACK_GAIN, "force_feedback_gain");

            auto mass_data = toml::find<std::vector<double>>(config, "link_mass_vector");
            link_mass_vector_ << mass_data[0], mass_data[1];
            validate_vec(link_mass_vector_, "link_mass_vector");

            auto length_data = toml::find<std::vector<double>>(config, "link_length_vector");
            link_length_vector_ << length_data[0], length_data[1];
            validate_vec(link_length_vector_, "link_length_vector");

            auto com_data = toml::find<std::vector<double>>(config, "dist_to_com_vector");
            dist_to_com_vector_ << com_data[0], com_data[1];
            validate_vec(dist_to_com_vector_, "dist_to_com_vector");
            validate_range(dist_to_com_vector_(0), 0.0, link_length_vector_(0), "dist_to_com_vector[0]");
            validate_range(dist_to_com_vector_(1), 0.0, link_length_vector_(1), "dist_to_com_vector[1]");

            max_actuator_torque_ = toml::find<int>(config, "max_actuator_torque");
            validate_range(max_actuator_torque_, 0, MAX_SENSOJOINT_TORQUE, "max_actuator_torque");

            control_loop_duration_ = toml::find<int>(config, "control_loop_duration");
            validate_range(control_loop_duration_, MIN_CONTROL_LOOP_DURATION, MAX_CONTROL_LOOP_DURATION, "control_loop_duration");
        } catch (const std::exception& e) {
            throw std::runtime_error("Missing impedance controller parameter in TOML: " + std::string(e.what()));
        }
    }

    void require_condition(const bool condition, const std::string& message) {
        if (!condition) throw std::runtime_error(message);
    }

    void validate_vec(const Eigen::Vector2d& vec, const std::string& name) {
        require_condition(vec.array().isFinite().all(),
                          name + " must contain finite values.");
        require_condition((vec.array() >= 0.0).all(), name + " elements must be >= 0.");
    }

    void validate_mat(const Eigen::Matrix2d& mat, const std::string& name) {
        require_condition(mat.array().isFinite().all(), name + " must contain finite values.");

        const double max_asymmetry = (mat - mat.transpose()).cwiseAbs().maxCoeff();
        require_condition(max_asymmetry < 1e-10, name + " must be symmetric.");
    
        Eigen::Vector2d eigenvalues = mat.selfadjointView<Eigen::Lower>().eigenvalues();
        require_condition((eigenvalues.array() >= -1e-10).all(),
                          name + " must be positive semi-definite (all eigenvalues >= 0).");
    }

    template <typename T>
    void validate_range(const T value, const T min_value, const T max_value, const std::string& name) {
        require_condition(value >= min_value && value <= max_value,
                          name + " out of bounds. Expected in [" + std::to_string(min_value) + ", " + std::to_string(max_value) + "].");
    }
}
