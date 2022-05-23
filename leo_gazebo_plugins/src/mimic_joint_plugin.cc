#ifndef _MOVE_LEG_NO_STEER_PLUGIN_HH_
#define _MOVE_LEG_NO_STEER_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "rclcpp/rclcpp.hpp"

namespace gazebo
{
/// \brief A plugin to control a Velodyne sensor.
class MimicJointPlugin : public ModelPlugin
{
        public: 
        
        /// \brief Constructor
        MimicJointPlugin() {}

        /// \brief The load function is called by Gazebo when the plugin is
        /// inserted into simulation
        /// \param[in] _model A pointer to the model that this plugin is
        /// attached to.
        /// \param[in] _sdf A pointer to the plugin's SDF element.
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {

                if (_model->GetJointCount() == 0)
                {
                std::cerr << "Invalid joint count, plugin not loaded\n";
                return;
                }

                // Check that the leg_id element exists, then read the value
                if (_sdf->HasElement("joint"))
                        joint_name = _sdf->Get<std::string>("joint");


                // Check that the leg_id element exists, then read the value
                if (_sdf->HasElement("mimic_joint"))
                        mimic_joint_name = _sdf->Get<std::string>("mimic_joint");

                // Check for sensitiveness element
                sensitiveness_ = 0.0;
                if (_sdf->HasElement("sensitiveness"))
                    sensitiveness_ = _sdf->GetElement("sensitiveness")->Get<double>();

                std::cerr << "joint_name: " << joint_name << "\n";
                std::cerr << "mimic_joint: " << mimic_joint_name << "\n";

                // Store the model pointer for convenience.
                this->model = _model;

                joint_ = this->model->GetJoint(joint_name);

                mimic_joint_ = this->model->GetJoint(mimic_joint_name);
                
                update_connection_ = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&MimicJointPlugin::UpdateChild, this));

        }


        
        void UpdateChild()
        {

                // Set mimic joint's angle based on joint's angle
                double angle = joint_->Position(0);
                double a = mimic_joint_->Position(0);



                if (fabs(angle - a) >= sensitiveness_) {
                        mimic_joint_->SetPosition(0, angle, true);
                    
                }
        }

        private: 
        /// \brief Pointer to the model.
        physics::ModelPtr model;

        std::string joint_name;
        std::string mimic_joint_name;

        /// \brief Pointer to the joints vector.
        physics::JointPtr joint_;
        physics::JointPtr mimic_joint_;

        double sensitiveness_;

        // Pointer to the update event connection
        event::ConnectionPtr update_connection_;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(MimicJointPlugin)

}
#endif