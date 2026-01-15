#include <stdexcept>
#include "impedance_controller_params/impedance_controller_params.hpp"
#include "impedance_controller_params/toml.hpp"

namespace impedance_controller_params {
    impedance_controller_params::params_t(const std::string& toml_path) {
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
            
            auto damping_data = toml::find<std::vector<std::vector<double>>>(config, "virtual_damping_matrix");
            virtual_damping_matrix_ << damping_data[0][0], damping_data[0][1],
                                       damping_data[1][0], damping_data[1][1];
            
            auto stiffness_data = toml::find<std::vector<std::vector<double>>>(config, "virtual_stiffness_matrix");
            virtual_stiffness_matrix_ << stiffness_data[0][0], stiffness_data[0][1],
                                         stiffness_data[1][0], stiffness_data[1][1];
            
            force_feedback_gain_ = toml::find<double>(config, "force_feedback_gain");
        } catch (const std::exception& e) {
            throw std::runtime_error("Missing impedance controller parameter in TOML: " + std::string(e.what()));
        }
    }
}
