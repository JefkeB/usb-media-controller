#ifndef USB_HID_KEYBOARD_CODES_INCLUDED
#define USB_HID_KEYBOARD_CODES_INCLUDED

// USB media codes
#define USB_HID_SCAN_NEXT 				0x01
#define USB_HID_SCAN_PREV 				0x02
#define USB_HID_STOP      				0x04
#define USB_HID_EJECT     				0x08
#define USB_HID_PAUSE     				0x10
#define USB_HID_MUTE      				0x20
#define USB_HID_VOL_UP    				0x40
#define USB_HID_VOL_DEC   				0x80

// USB keyboard codes
#define USB_HID_MODIFIER_LEFT_CTRL   	0x01
#define USB_HID_MODIFIER_LEFT_SHIFT  	0x02
#define USB_HID_MODIFIER_LEFT_ALT    	0x04
#define USB_HID_MODIFIER_LEFT_GUI    	0x08 // (Win/Apple/Meta)
#define USB_HID_MODIFIER_RIGHT_CTRL 	0x10
#define USB_HID_MODIFIER_RIGHT_SHIFT 	0x20
#define USB_HID_MODIFIER_RIGHT_ALT   	0x40
#define USB_HID_MODIFIER_RIGHT_GUI   	0x80
#define USB_HID_KEY_L     				0x0F

#endif
