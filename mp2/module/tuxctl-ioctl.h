// All necessary declarations for the Tux Controller driver must be in this file

#ifndef TUXCTL_H
#define TUXCTL_H

#define TUX_SET_LED _IOR('E', 0x10, unsigned long)
#define TUX_READ_LED _IOW('E', 0x11, unsigned long*)
#define TUX_BUTTONS _IOW('E', 0x12, unsigned long*)
#define TUX_INIT _IO('E', 0x13)
#define TUX_LED_REQUEST _IO('E', 0x14)
#define TUX_LED_ACK _IO('E', 0x15)


#define OCCUPY 1
#define FREE 0
#define INIT_BUF_SIZE 2
#define LED_BUF_SIZE 6



#define BUTTON_PACKET_SIZE 2
#define INIT_BUF_SIZE   2
#define DISPLAY_SIZE    4
#define DOT_MASK		    0x10
#define LED_PACKET_SIZE 6

#define ACK_ON          0
#define ACK_OFF         1


#define MIN_LED_BYTES   2


#endif

