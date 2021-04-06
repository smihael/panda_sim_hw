// Copyright (c) 2017 Franka Emika GmbH
// Copyright (c) 2021 Jozef Stefan Institute


// Use of this source code is governed by the Apache-2.0 license, see LICENSE


#pragma once

#include <array>
#include <string>

#include <franka/model.h>
//#include <franka/robot_state.h>
#include <hardware_interface/internal/hardware_resource_manager.h>

#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>


namespace franka_hw {

/**
 * Handle to perform calculations using the dynamic model of a robot.
 */
class FrankaModelHandle {
 public:
  FrankaModelHandle() = delete;


  /**
   * Creates an instance of a FrankaModelHandle.
   *
   * @param[in] name The name of the model handle.
   * @param[in] model A reference to the franka::Model instance wrapped by this handle.
   * @param[in] robot_state A reference to the current robot state.
   */
  //FrankaModelHandle(const std::string& name, franka::Model& model, franka::RobotState& robot_state)
  //    : name_(name), model_(&model), robot_state_(&robot_state) {}
  FrankaModelHandle(const std::string& name,
   const std::array<double, 42>& zeroJacobian,
   //const std::array<double, 16>& pose,
   const std::array<double, 7>&  gravity,
   const std::array<double, 7>& coriolis,
   const std::array<double, 49>& mass)  :
   name_(name),
   zeroJacobian_(&zeroJacobian),
   //pose_(&pose),
   gravity_(&gravity),
   coriolis_(&coriolis),
   mass_(&mass)   {}

  /**
   * Gets the name of the model handle.
   *
   * @return Name of the model handle.
   */
  std::string getName() const noexcept { return name_; }

/**
   * Calculates the 7x7 mass matrix from the current robot state. Unit: \f$[kg \times m^2]\f$.
   *
   * @return Vectorized 7x7 mass matrix, column-major.
   *
   * @see franka::Model::mass
   */
  std::array<double, 49> getMass() const { return *mass_; }


  /**
   * Calculates the Coriolis force vector (state-space equation) from the current robot state:
   * \f$ c= C \times dq\f$, in \f$[Nm]\f$.
   *
   * @return Coriolis force vector.
   *
   * @see franka::Model::coriolis
   */
  std::array<double, 7> getCoriolis() const { return *coriolis_; }

  /**
   * Calculates the gravity vector from the current robot state. Unit: \f$[Nm]\f$.
   *
   * @return Gravity vector.
   *
   * @see franka::Model::gravity
   */
  std::array<double, 7> getGravity() const {
    return *gravity_;
  }

  /**
   * Gets the 6x7 Jacobian for the given joint relative to the base frame.
   *
   * The Jacobian is represented as a 6x7 matrix in column-major format and calculated from
   * the current robot state.
   *
   * @param[in] frame The desired frame.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   *
   * @see franka::Model::zeroJacobian
   */
  std::array<double, 42> getZeroJacobian(const franka::Frame& frame) const {
    return *zeroJacobian_;
  }

  /* Not implemented:
  std::array<double, 7> getCoriolis(const std::array<double, 7>& q, const std::array<double, 7>& dq, const std::array<double, 9>& total_inertia, double total_mass, const std::array<double, 3>& F_x_Ctotal) const;
  std::array<double, 7> getGravity(const std::array<double, 7>& q, double total_mass, const std::array<double, 3>& F_x_Ctotal,  const std::array<double, 3>& gravity_earth = {{0., 0., -9.81}});
  std::array<double, 16> getPose(const franka::Frame& frame) const;
  std::array<double, 16> getPose(const franka::Frame& frame, const std::array<double, 7>& q, const std::array<double, 16>& F_T_EE, const std::array<double, 16>& EE_T_K) const;
  std::array<double, 42> getBodyJacobian(const franka::Frame& frame) const;
  std::array<double, 42> getBodyJacobian(const franka::Frame& frame, const std::array<double, 7>& q, const std::array<double, 16>& F_T_EE, const std::array<double, 16>& EE_T_K);
  std::array<double, 42> getZeroJacobian(const franka::Frame& frame, const std::array<double, 7>& q, const std::array<double, 16>& F_T_EE, const std::array<double, 16>& EE_T_K) const;
  std::array<double, 49> getMass(const std::array<double, 7>& q, const std::array<double, 9>& total_inertia, double total_mass, const std::array<double, 3>& F_x_Ctotal) const;
  */

 private:
  std::string name_;
  //const franka::RobotState* robot_state_;
  const std::array<double, 42>* zeroJacobian_;
  //const std::array<double, 16>* pose_;
  const std::array<double, 7>*  gravity_;
  const std::array<double, 7>* coriolis_;
  const std::array<double, 49>* mass_;

};

/**
 * Hardware interface to perform calculations using the dynamic model of a robot.
 */
class FrankaModelInterface : public hardware_interface::HardwareResourceManager<FrankaModelHandle> {
};

}  // namespace franka_hw
