// ****************************************************************************
// Title		: Debounce
// File Name	: 'Debounce.cpp'
// Target MCU	: Espressif ESP32 (Doit DevKit Version 1)
//
// Code based on
// https://hackaday.com/2015/12/10/embed-with-elliot-debounce-your-noisy-buttons-part-ii/
//
// Revision History:
// When			Who			Description of change
// -----------	-----------	-----------------------
// 14-APR-2022	brooks		program start
// 27-MAR-2025	brooks		updated to 16-bit history per article
// ****************************************************************************

// Include Files
// ****************************************************************************
#include <Arduino.h>
#include "Debounce.h"

// 16-bit masks and patterns
#define MASK        0b1111000000111111    // Don't care bits = 0
#define RELEASE     0b1111000000000000    // Button released pattern
#define DOWN        0b1111111111111111    // Button down mask
#define UP          0b0000000000000000    // Button up
#define PRESS       0b0000000000111111    // Button pressed pattern
#define CLEAR       0b0000000000000000    // Clear history
#define SET         0b1111111111111111    // Set history

// Constructors
// ****************************************************************************
Debounce::Debounce(uint8_t buttonPin)		// Pin to debounce
{
    _button = buttonPin;
	_active = HIGH;
    _buttonHistory = 0;                    // Initialize history to zero (button up)
    _prevState = 0;                        // Initialize previous state
}

Debounce::Debounce(uint8_t buttonPin, uint8_t logicLevel)
{
    _button = buttonPin;
	_active = logicLevel;					// LOW or HIGH
    
    // If active level is LOW, initialize history to all 1s (button up)
    // If active level is HIGH, initialize history to all 0s (button up)
    _buttonHistory = (_active == LOW) ? 0xFFFF : 0x0000;
    _prevState = 0;                        // Initialize previous state
}


// Functions
// ****************************************************************************
uint8_t Debounce::readButton(void)
{
	uint8_t result = false;
	if(_active == LOW && digitalRead(_button) == LOW)
	{
		result = true;
	}
	else if(_active == HIGH && digitalRead(_button) == HIGH)
	{
		result = true;
	}
    return result;
}

void Debounce::update(void)
{
    _buttonHistory = (_buttonHistory << 1);  // Shift left
    _buttonHistory |= readButton();          // Add new reading
    
    // Update the previous state for state change detection
    _prevState = isDown();
}

uint8_t Debounce::isPressed(void)
{
    uint8_t _pressed = 0;

	if ((_buttonHistory & MASK) == PRESS)
	{
		_pressed = 1;
		_buttonHistory = SET;  // Set history to all 1s after press detected
	}
    return _pressed;
}

uint8_t Debounce::isReleased(void)
{
    uint8_t _released = 0;

	if ((MASK & _buttonHistory) == RELEASE)
	{
		_released = 1;
		_buttonHistory = CLEAR;  // Clear history to all 0s after release detected
	}
    return _released;
}

uint8_t Debounce::isDown(void)
{
	return (_buttonHistory == DOWN);
}

uint8_t Debounce::isUp(void)
{
	return (_buttonHistory == UP);
}

uint8_t Debounce::stateChanged(void)
{
    uint8_t currentState = isDown();
    uint8_t changed = (currentState != _prevState);
    return changed;
}