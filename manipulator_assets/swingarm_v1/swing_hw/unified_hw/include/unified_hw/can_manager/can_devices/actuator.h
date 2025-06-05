//
// Created by lsy on 25-6-4.
//

// Actuator.h

#pragma once
#include <string>
#include <stdexcept>

class ActuatorCoefficients {
public:
  double act2pos;
  double act2vel;
  double act2effort;
  double kp2act;
  double kd2act;
  double pos2act;
  double vel2act;
  double effort2act;
  double pos_offset;
  double vel_offset;
  double effort_offset;

  ActuatorCoefficients()
      : act2pos(0.), act2vel(0.), act2effort(0.),
        kp2act(0.), kd2act(0.), pos2act(0.),
        vel2act(0.), effort2act(0.),
        pos_offset(0.), vel_offset(0.), effort_offset(0.) {}

  ActuatorCoefficients(
      double act2pos, double act2vel, double act2effort,
      double kp2act, double kd2act, double pos2act,
      double vel2act, double effort2act,
      double pos_offset, double vel_offset, double effort_offset)
      : act2pos(act2pos)
        , act2vel(act2vel)
        , act2effort(act2effort)
        , kp2act(kp2act)
        , kd2act(kd2act)
        , pos2act(pos2act)
        , vel2act(vel2act)
        , effort2act(effort2act)
        , pos_offset(pos_offset)
        , vel_offset(vel_offset)
        , effort_offset(effort_offset) {}

  void printCoefficients() const {
    std::cout << "---- ActuatorCoefficients ----\n";
    std::cout << " act2pos:      " << act2pos   << "\n";
    std::cout << " act2vel:      " << act2vel   << "\n";
    std::cout << " act2effort:   " << act2effort<< "\n";
    std::cout << " kp2act:       " << kp2act    << "\n";
    std::cout << " kd2act:       " << kd2act    << "\n";
    std::cout << " pos2act:      " << pos2act   << "\n";
    std::cout << " vel2act:      " << vel2act   << "\n";
    std::cout << " effort2act:   " << effort2act<< "\n";
    std::cout << " pos_offset:   " << pos_offset << "\n";
    std::cout << " vel_offset:   " << vel_offset << "\n";
    std::cout << " effort_offset:" << effort_offset << "\n";
  }

  inline double actuator2pos(uint16_t raw_act) const {
    return act2pos * static_cast<double>(raw_act) + pos_offset;
  }
  inline double actuator2vel(uint16_t raw_act) const {
    return act2vel * static_cast<double>(raw_act) + vel_offset;
  }
  inline double actuator2effort(uint16_t raw_act) const {
    return act2effort * static_cast<double>(raw_act) + effort_offset;
  }

  inline uint16_t pos2actuator(double phys_pos) const {
    double tmp = pos2act * (phys_pos - pos_offset);
    return static_cast<uint16_t>(tmp);
  }
  inline uint16_t vel2actuator(double phys_vel) const {
    double tmp = vel2act * (phys_vel - vel_offset);
    return static_cast<uint16_t>(tmp);
  }
  inline uint16_t effort2actuator(double phys_effort) const {
    double tmp = effort2act * (phys_effort - effort_offset);
    return static_cast<uint16_t>(tmp);
  }

  inline uint16_t kpPhys2actuator(double phys_kp) const {
    return static_cast<uint16_t>(kp2act * phys_kp);
  }
  inline uint16_t kdPhys2actuator(double phys_kd) const {
    return static_cast<uint16_t>(kd2act * phys_kd);
  }
};

