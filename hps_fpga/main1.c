/*
This program demonstrate how to use hps communicate with FPGA through light AXI Bridge.
uses should program the FPGA by GHRD project before executing the program
refer to user manual chapter 7 for details about the demo
*/


#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "hwlib.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_gpio.h"
#include "hps_0.h"

#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

int main() {

	void *virtual_base;
	int fd;
	int loop_count;
	int led_direction;
	int led_mask;
	void *h2p_lw_led_addr;
	int result;

	// map the address space for the LED registers into user space so we can interact with them.
	// we'll actually map in the entire CSR span of the HPS since we want to access various registers within that span

	if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
		printf( "ERROR: could not open \"/dev/mem\"...\n" );
		return( 1 );
	}

	virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );

	if( virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap() failed...\n" );
		close( fd );
		return( 1 );
	}
	/*ALT_STM_OFST https://git.flirble.org/chrisy/freertos/blob/b5c8097ae5b521c10bd945657325068da3af1450/FreeRTOS/Demo/CORTEX_A9_Cyclone_V_SoC_DK/Altera_Code/HardwareLibrary/include/socal/hps.h  */
	/* ALT_LWFPGASLVS_OFST https://git.flirble.org/chrisy/freertos/blob/b5c8097ae5b521c10bd945657325068da3af1450/FreeRTOS/Demo/CORTEX_A9_Cyclone_V_SoC_DK/Altera_Code/HardwareLibrary/include/socal/hps.h	*/
	// LED_PIO_BASE in hps_0.h (qsys)
	// LED_PIO_DATA_WIDTH in hps_0.h (qsys)
	h2p_lw_led_addr=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + LED_PIO_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	

	// toggle the LEDs a bit
	
	int v_teste[2] = {1,2};
	
	for (int i = 0; i < 2; i++) {
		if (*endereço_mem & 0x1) {
			// escreve na memória
			*(uint32_t *)h2p_lw_led_addr = v_teste[i];
			// espera
			while (1) {
				if (! *endereço_mem & 0x2) {
					break;
				}
			}
		}
	}
	
	if (*endereço_mem & 0x2){
		result = *(uint32_t *)h2p_lw_led_addr;
		printf("%d\n", result);
	}

	// clean up our memory mapping and exit
	
	if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
		printf( "ERROR: munmap() failed...\n" );
		close( fd );
		return( 1 );
	}

	close( fd );

	return( 0 );
}
