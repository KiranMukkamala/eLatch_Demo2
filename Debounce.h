// ****************************************************************************
// Title		: Debounce
// File Name	: 'Debounce.h'
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

#ifndef DEBOUNCE_H
#define DEBOUNCE_H


class Debounce
{
	public:
		// Constructors
		Debounce(uint8_t buttonPin);		// Default active HIGH logic

		Debounce(uint8_t buttonPin, uint8_t activeLevel);

		void update(void);
		uint8_t isPressed(void);
		uint8_t isUp(void);
		uint8_t isDown(void);
		uint8_t isReleased(void);
		uint8_t stateChanged(void);

	protected:

	private:
		uint16_t _buttonHistory=0;     // Changed to 16-bit history
		uint8_t _button=0;
		uint8_t _active=0;
		uint8_t _prevState=0;          // Track previous button state

		uint8_t readButton(void);
};

#endif										// DEBOUNCE_H