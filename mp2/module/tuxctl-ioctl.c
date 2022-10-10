/* tuxctl-ioctl.c
 *
 * Driver (skeleton) for the mp2 tuxcontrollers for ECE391 at UIUC.
 *
 * Mark Murphy 2006
 * Andrew Ofisher 2007
 * Steve Lumetta 12-13 Sep 2009
 * Puskar Naha 2013
 */

#include <asm/current.h>
#include <asm/uaccess.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/file.h>
#include <linux/miscdevice.h>
#include <linux/kdev_t.h>
#include <linux/tty.h>
#include <linux/spinlock.h>

#include "tuxctl-ld.h"
#include "tuxctl-ioctl.h"
#include "mtcp.h"

#define debug(str, ...) \
	printk(KERN_DEBUG "%s: " str, __FUNCTION__, ## __VA_ARGS__)

/************************ Protocol Implementation *************************/
int ack;
static spinlock_t mtx_ack = SPIN_LOCK_UNLOCKED;

unsigned char INIT_buf[INIT_BUF_SIZE];
unsigned char LED_buf[LED_BUF_SIZE];
unsigned char BUTTON_buf;
unsigned char button_information[2];

int INIT(struct tty_struct* tty);
int SET_LED(struct tty_struct*tty,unsigned long arg);
int SET_BUTTONS(struct tty_struct*tty,unsigned long arg);

unsigned char prev_state [LED_PACKET_SIZE];


/* tuxctl_handle_packet()
 * IMPORTANT : Read the header for tuxctl_ldisc_data_callback() in 
 * tuxctl-ld.c. It calls this function, so all warnings there apply 
 * here as well.
 */
void tuxctl_handle_packet (struct tty_struct* tty, unsigned char* packet)
{
    unsigned a, b, c;

    a = packet[0]; /* Avoid printk() sign extending the 8-bit */
    b = packet[1]; /* values when printing them. */
    c = packet[2];

	switch(a){
		case MTCP_RESET:
			INIT(tty);
			//send the led value before reset.
			tuxctl_ldisc_put(tty,LED_buf,6);
			break;

		case MTCP_BIOC_EVENT:
			button_information[0] = b; // 1 x x x C B A START
			button_information[1] = c; // 1 x x x R D L U
			break;

		case MTCP_ACK:
			spin_lock(&mtx_ack);
			ack = FREE;            
			spin_unlock(&mtx_ack);
			break;

		default:
			break;
		
	}
    /*printk("packet : %x %x %x\n", a, b, c); */
}

/******** IMPORTANT NOTE: READ THIS BEFORE IMPLEMENTING THE IOCTLS ************
 *                                                                            *
 * The ioctls should not spend any time waiting for responses to the commands *
 * they send to the controller. The data is sent over the serial line at      *
 * 9600 BAUD. At this rate, a byte takes approximately 1 millisecond to       *
 * transmit; this means that there will be about 9 milliseconds between       *
 * the time you request that the low-level serial driver send the             *
 * 6-byte SET_LEDS packet and the time the 3-byte ACK packet finishes         *
 * arriving. This is far too long a time for a system call to take. The       *
 * ioctls should return immediately with success if their parameters are      *
 * valid.                                                                     *
 *                                                                            *
 ******************************************************************************/
int 
tuxctl_ioctl (struct tty_struct* tty, struct file* file, 
	      unsigned cmd, unsigned long arg)
{
    switch (cmd) {
	
	case TUX_INIT:
		INIT(tty);
		return 0;
	
	case TUX_BUTTONS:
		return SET_BUTTONS(tty,arg);
	
	case TUX_SET_LED:
		return SET_LED(tty,arg);
	
	case TUX_LED_ACK:
	case TUX_LED_REQUEST:
	case TUX_READ_LED:
	default:
	    return -EINVAL;
    }
}

/*
 * INIT
 * DESCRIPTION: initialize the tux and pop a buffer with MTCP_BIOC_ON and MTCP_LED_USR
 * INPUTS: tty
 * OUTPUTS: none
 * RETURN VALUE: none
 * SIDE EFFECTS: initialize the tux 
 */
int INIT(struct tty_struct* tty){            
	spin_lock(&mtx_ack);
	ack = OCCUPY;            
	spin_unlock(&mtx_ack);
	INIT_buf[0] = MTCP_BIOC_ON;
	INIT_buf[1] = MTCP_LED_USR;
	tuxctl_ldisc_put(tty,INIT_buf,INIT_BUF_SIZE);
	return 0;
}

/*
 * SET_LED
 * DESCRIPTION: takes 4 bytes arg and do the calculation to be a 6 bytes package and send it to TUX to set LED.
 * INPUTS: tty,arg(4 bytes)
 * OUTPUTS: none
 * RETURN VALUE: none
 * SIDE EFFECTS: set the tux led
 */

int SET_LED(struct tty_struct*tty,unsigned long arg){
	//arg: dot_index led_index number number
	//LED_buf: opcode led_index led0 led1 led2 led3
	int i;
	unsigned char led[16] ={0xE7, 0x06, 0xCB, 0x8F, 0x2E, 0xAD, 0xED, 0x86, 0xEF, 0xAE, 0xEE, 0x6D, 0xE1, 0x4F, 0xE9, 0xE8};// this is the LED value for 0,1,2,3,4,5,6,7,8,9,A,B,C,D,E,F with no dot.
	unsigned char LED_index;
	unsigned char DOT_index;
	unsigned char HEX_value[4];
	
	//check whether the ack is busy if so skip the function.
    spin_lock(&mtx_ack);
	if(ack == OCCUPY){
		spin_unlock(&mtx_ack);
		return 0;
	}
	ack = OCCUPY;
	spin_unlock(&mtx_ack);

//shift the arg to get proper information
	LED_buf[0] = MTCP_LED_SET;
	LED_index = (arg >> 16) & (0x0F); // 0 0 0 0 LED3 LED2 LED1 LED0 
	DOT_index = (arg >> 24) & (0x0F); // 0 0 0 0 DOT3 DOT2 DOT1 DOT0
	HEX_value[0] = (arg & 0xFFFF) & 0x0F; // LED0 value
	HEX_value[1] = ((arg & 0xFFFF) >> 4) & 0x0F;// LED1 value
	HEX_value[2] = ((arg & 0xFFFF) >> 8) & 0x0F;//LED2 value
	HEX_value[3] = ((arg & 0xFFFF) >> 12) & 0x0F;//LED3 value

	for(i = 0;i < 4;i++){
		//Write the hex value
		if(((LED_index >> i) & 0x01) == 1){ // check the LED_index
			LED_buf[i+2] = led[HEX_value[i]]; // since the index is inverse.
		}
		else{
			LED_buf[i+2] = 0;
		}

		if(((DOT_index >> i) & 0x01) == 1){
			LED_buf[i+2]  = LED_buf[i+2] | 0x10;
		}
	}
	LED_buf[1] = 0x0F; // LET ALL THE LIGHTS ON
	tuxctl_ldisc_put(tty,LED_buf,6);
	return 0;
}

/*
 * SET_BUTTONS
 * DESCRIPTION: takes the packets value and reorder it into a new data which is send to the user for new behavior. 
 * INPUTS: tty,arg(4 bytes)
 * OUTPUTS: none
 * RETURN VALUE: none
 * SIDE EFFECTS: send the button information
 */

int SET_BUTTONS(struct tty_struct*tty,unsigned long arg){
	unsigned char CBAS;
	unsigned char RDLU;
	unsigned char D;
	unsigned char L;
	unsigned char RLDU;
	int flag;

	CBAS = button_information[0] & 0x0F;
	RDLU = button_information[1] & 0x0F;

	// change RDLU into RLDU
	D = (RDLU & 0x04) >> 1;
	L = (RDLU & 0x02) << 1;

	RDLU = RDLU & 0x09;        // 0000 1001
	
	RLDU = RDLU | D | L;
	
	BUTTON_buf = (RLDU << 4) | CBAS;
	
	flag = copy_to_user((unsigned long*)arg, &BUTTON_buf, sizeof(BUTTON_buf));
	if (flag != 0){
		return -EINVAL;
	}
	else{
		return 0;
	}

}





	






