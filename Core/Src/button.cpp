/*
 * button.cpp
 *
 *  Created on: Jan 10, 2023
 *      Author: drCsabesz
 */

#include "button.h"

void Button::Update( bool value )
{
  switch( this->currState )
  {
    case Off:
    {
      if( value == this->activeState )
      {
        if( this->debounceTh > this->debounceCtr )
        {
          this->debounceCtr++;
        }
        else
        {
          this->currState = Pressed;
          this->debounceCtr = 0u;
        }
      }
      else
      {
        if( this->debounceCtr > 0u )
        {
          this->debounceCtr--;
        }
      }
      break;
    }
    case Pressed:
    {
      this->currState = Hold;
      break;
    }
    case Hold:
    {
      if( value != this->activeState )
      {
        if( this->debounceTh > this->debounceCtr )
        {
          this->debounceCtr++;
        }
        else
        {
          this->currState = Released;
          this->debounceCtr = 0u;
        }
      }
      else
      {
        if( this->debounceCtr > 0u )
        {
          this->debounceCtr--;
        }
      }
      break;
    }
    case Released:
    default:
    {
      this->currState = Off;
      break;
    }
  }
}


