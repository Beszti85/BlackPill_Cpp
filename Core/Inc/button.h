/*
 * button.cpp
 *
 *  Created on: Jan 10, 2023
 *      Author: drCsabesz
 */

#ifndef INC_BUTTON_CPP_
#define INC_BUTTON_CPP_

#include "stdint.h"

enum ButtonState
{
  Off = 0u,
  Pressed,
  Hold,
  Released
};

class Button
{
public:
  Button( bool actState, uint16_t debValue ) { activeState = actState; debounceTh = debounceCtr; };
  void Update( bool value );
  bool IsPressed( void )  { return isPressed; };
  bool IsReleased( void ) { return isReleased; };
  ButtonState CurrState( void )  { return currState; };
private:
  uint16_t      debounceTh;
  uint16_t      debounceCtr;
  bool          activeState;
  bool          isPressed;
  bool          isReleased;
  ButtonState   currState;
};

#endif /* INC_BUTTON_CPP_ */
