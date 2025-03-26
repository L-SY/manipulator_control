//
// Created by lsy on 25-3-26.
//

#pragma once

#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/hardware_interface.h>

namespace hardware_interface
{

/**
 * @brief Handle class for button panel devices, providing access to button states
 */
class ButtonPanelHandle
{
public:
  ButtonPanelHandle() = default;

  /**
   * @brief Constructor
   * @param name Name of the button panel
   * @param button1_state Pointer to button1 state
   * @param button2_state Pointer to button2 state
   */
  ButtonPanelHandle(const std::string& name, const bool* button1_state, const bool* button2_state)
      : name_(name)
        , button1_state_(button1_state)
        , button2_state_(button2_state)
  {
    if (!button1_state)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. Button1 state pointer is null.");
    if (!button2_state)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. Button2 state pointer is null.");
  }

  /**
   * @brief Get the name of the button panel
   * @return Button panel name
   */
  std::string getName() const
  {
    return name_;
  }

  /**
   * @brief Get the state of button1
   * @return true if pressed, false if released
   */
  bool getButton1State() const
  {
    assert(button1_state_);
    return *button1_state_;
  }

  /**
   * @brief Get the state of button2
   * @return true if pressed, false if released
   */
  bool getButton2State() const
  {
    assert(button2_state_);
    return *button2_state_;
  }

private:
  std::string name_;
  const bool* button1_state_{nullptr};
  const bool* button2_state_{nullptr};
};

/**
 * @brief Hardware interface for button panels, using DontClaimResources policy
 *        to allow multiple controllers to access simultaneously
 */
class ButtonPanelInterface
    : public hardware_interface::HardwareResourceManager<ButtonPanelHandle, hardware_interface::DontClaimResources>
{
};

} // namespace hardware_interface