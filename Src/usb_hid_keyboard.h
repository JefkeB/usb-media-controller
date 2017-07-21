/*
 * usb_hid_keyboard.h
 *
 *  Created on: 21 jul. 2017
 *      Author: JefPC
 */

#ifndef USB_HID_KEYBOARD_H_
#define USB_HID_KEYBOARD_H_


#include "usb_hid_keyboard_codes.h"

// HID Keyboard
//
struct keyboardHID_t {
	uint8_t id;
	uint8_t modifiers;
	uint8_t key1;
	uint8_t key2;
	uint8_t key3;
};


// HID Media
//
struct mediaHID_t {
	uint8_t id;
	uint8_t keys;
};

#endif /* USB_HID_KEYBOARD_H_ */
