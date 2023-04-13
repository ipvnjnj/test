#include "gd32f10x_i2c.h"
#include "gd32f10x_eval.h"
#include "i2c_msg.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "systick.h"
#include "I2C_IE.h"
/*
Bit0~3: port number
Bit4~5: port status
  00: disconnect
  01: connect
  10: bypass
  11: reserved
Bit6~7: 00
*/
Port_Curr portCurr;
Port_Watchdog portWatchdog;
Port_PowerOff portPowerOff;
Port_NextPowerUP portNextPowerUP;
EEPROM_FLAG eeprom_flag;


int get_power_state(uint8_t *state)
{
	char power_state = 0;
		
	power_state = gpio_input_bit_get(POWER_STATUS_PORT, POWER_STATUS_PIN);
	
	*state = power_state;
	return 0;
}

int read_eeprom_state(uint16_t addr, uint8_t *data, uint8_t len)
{
	int i;
	eeprom_wr = 1;
	ipmb_a_txData[0] = 0;
	ipmb_a_txData[1] = (uint8_t)addr;
	I2C_nBytes = 2;
	/* the master waits until the I2C bus is idle */
	while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY));
	/* the master sends a start condition to I2C bus */
	i2c_start_on_bus(I2C0);

	delay_1ms(10);
	/* the master waits until the I2C bus is idle */
	while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY));
	/* the master sends a start condition to I2C bus */
	eeprom_wr = 0;
	I2C_nBytes = len;
	i2c_start_on_bus(I2C0);
	delay_1ms(10);
	while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY));
	for( i = 0; i < len; i ++ )
	{
		data[i] = ipmb_a_rxData[i];
	}
	return 0;
}

int write_eeprom_state(uint16_t addr, uint8_t *data, uint8_t len)
{
	int i;
	
	eeprom_wr = 1;
	ipmb_a_txData[0] = 0;
	ipmb_a_txData[1] = addr;
	I2C_nBytes = len + 2;

	for( i = 0; i < len; i ++ )
	{
		ipmb_a_txData[i+2] = data[i];
	}
	/* the master waits until the I2C bus is idle */
	while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY));
	/* the master sends a start condition to I2C bus */
	i2c_start_on_bus(I2C0);
	delay_1ms(10);
	return 0;
}


static int set_led_state_one(uint32_t gpio_addr, uint32_t gpio_pin, uint8_t state,uint8_t pin_state)
{
	if (state == CONNECT)
	{
		GPIO_BC(gpio_addr) = gpio_pin;
	}
	else if (state == BYPASS)
	{
		GPIO_BOP(gpio_addr) = gpio_pin;
	}
	else if (state == DISCONNECT)
	{
		if (pin_state == 0)
			GPIO_BC(gpio_addr) = gpio_pin;
		else
			GPIO_BOP(gpio_addr) = gpio_pin;
	}
	return EXIT_SUCCESS;
}

int set_led_state(uint8_t pin_state)
{
	uint8_t state;
	uint32_t gpio_addr, gpio_pin;
	
	gpio_addr = GPIOA;
	gpio_pin = GPIO_PIN_1;
	state = portCurr.port_info[2] >> 4;
	set_led_state_one(gpio_addr, gpio_pin, state, pin_state);

	gpio_addr = GPIOA;
	gpio_pin = GPIO_PIN_6;
	state = portCurr.port_info[0] >> 4;
	set_led_state_one(gpio_addr, gpio_pin, state, pin_state);

	gpio_addr = GPIOB;
	gpio_pin = GPIO_PIN_8;
	state = portCurr.port_info[4] >> 4;
	set_led_state_one(gpio_addr, gpio_pin, state, pin_state);
	
	return EXIT_SUCCESS;
}

static int set_port_state(uint8_t portNum, uint8_t portState)
{
	if (portNum == 0)
	{
		if (portState == CONNECT)
		{
			GPIO_BC(GPIOB) = GPIO_PIN_12;
			GPIO_BC(GPIOB) = GPIO_PIN_13;
			GPIO_BC(GPIOB) = GPIO_PIN_14;
		}
		else if (portState == DISCONNECT)
		{
			GPIO_BOP(GPIOB) = GPIO_PIN_12;
			GPIO_BC(GPIOB) = GPIO_PIN_13;
			GPIO_BOP(GPIOB) = GPIO_PIN_14;
		}
		else if (portState == BYPASS)
		{ 
			GPIO_BOP(GPIOB) = GPIO_PIN_12;
			GPIO_BOP(GPIOB) = GPIO_PIN_13;
			GPIO_BC(GPIOB) = GPIO_PIN_14;
		}
		else if (portState == RESERVED)
		{
			GPIO_BOP(GPIOB) = GPIO_PIN_12;
			GPIO_BC(GPIOB) = GPIO_PIN_13;
			GPIO_BC(GPIOB) = GPIO_PIN_14;
		}
	}
	else if (portNum == 1)
	{
		if (portState == CONNECT)
		{
			GPIO_BC(GPIOA) = GPIO_PIN_3;
			GPIO_BC(GPIOA) = GPIO_PIN_4;
			GPIO_BC(GPIOA) = GPIO_PIN_5;
		}
		else if (portState == DISCONNECT)
		{
			GPIO_BOP(GPIOA) = GPIO_PIN_3;
			GPIO_BC(GPIOA) = GPIO_PIN_4;
			GPIO_BOP(GPIOA) = GPIO_PIN_5;
		}
		else if (portState == BYPASS)
		{
			GPIO_BOP(GPIOA) = GPIO_PIN_3;
			GPIO_BOP(GPIOA) = GPIO_PIN_4;
			GPIO_BC(GPIOA) = GPIO_PIN_5;
		}
		else if (portState == RESERVED)
		{
			GPIO_BOP(GPIOA) = GPIO_PIN_3;
			GPIO_BC(GPIOA) = GPIO_PIN_4;
			GPIO_BC(GPIOA) = GPIO_PIN_5;
		}
	}
	else if (portNum == 2)
	{
		if (portState == CONNECT)
		{
			GPIO_BC(GPIOB) = GPIO_PIN_0;
			GPIO_BC(GPIOB) = GPIO_PIN_1;
			GPIO_BC(GPIOB) = GPIO_PIN_5;
		}
		else if (portState == DISCONNECT)
		{
			GPIO_BOP(GPIOB) = GPIO_PIN_0;
			GPIO_BC(GPIOB) = GPIO_PIN_1;
			GPIO_BOP(GPIOB) = GPIO_PIN_5;
		}
		else if (portState == BYPASS)
		{
			GPIO_BOP(GPIOB) = GPIO_PIN_0;
			GPIO_BOP(GPIOB) = GPIO_PIN_1;
			GPIO_BC(GPIOB) = GPIO_PIN_5;
		}
		else if (portState == RESERVED)
		{
			GPIO_BOP(GPIOB) = GPIO_PIN_0;
			GPIO_BC(GPIOB) = GPIO_PIN_1;
			GPIO_BC(GPIOB) = GPIO_PIN_5;
		}
	}
	
	return EXIT_SUCCESS;
}

int port_dump_saved_content(uint8_t *buff)
{
	int i = 0;
	uint8_t *tmp = buff;
	
	for (i=0;i<PORTNUM;i++)
	{
		*tmp = portCurr.port_info[i];
		tmp++;
	}
	
	for (i=0;i<PORTNUM;i++)
	{
		*tmp = portWatchdog.port_info[i];
		tmp++;
	}
	
	return EXIT_SUCCESS;
}

int port_read_from_flash(void)
{
	//int i;
	int retval = EXIT_SUCCESS; 
	uint8_t eeprom_data[20];

	read_eeprom_state(POWEROFF_ADDR_PORT_NUM, eeprom_data, 10);
	/*port number*/
	if (PORTNUM != eeprom_data[0])
		retval = EXIT_FAILURE;
	/*read port next power up state saved in flash*/
	memcpy(&portPowerOff.port_num, eeprom_data, sizeof(portPowerOff));
	memset(eeprom_data, 0, 20);
	
	read_eeprom_state(NEXT_POWERUP_ADDR_PORT_NUM, eeprom_data, 10);
	/*port number*/
	if (PORTNUM != eeprom_data[0])
		retval = EXIT_FAILURE;
	/*read port next power up state saved in flash*/
	memcpy(&portNextPowerUP.port_num, eeprom_data, sizeof(portNextPowerUP));
	
	/*
	portNextPowerUP.port_num = c_data[0];
	for (i=0; i<c_data[0]; i++)
	{
		portNextPowerUP.port_info[i] = c_data[i+1];;
	}
	*/	
	return retval;
}

int port_get_port_state(uint8_t portMode, uint8_t portNum, uint8_t *portState)
{
	int retval = EXIT_SUCCESS;
	switch (portMode)
	{
		case CURRENT:
			*portState = portCurr.port_info[portNum] >> 4; 
			break;
		
		case POWEROFF:
			*portState = portPowerOff.port_info[portNum] >> 4; 
			break;
		
		case NEXT_PWOERUP:
			*portState = portNextPowerUP.port_info[portNum] >> 4; 
			break;
		
		case WATCHDOG:
			*portState = portWatchdog.port_info[portNum] >> 4; 
			break;
		
		default:
			retval = EXIT_FAILURE;
			break;
	}
	
	return retval;
}

int port_set_port_state(uint8_t portMode, uint8_t portNum, uint8_t portState)
{
	int retval = EXIT_SUCCESS;
	uint8_t port_data[2];
	
	uint8_t portNum0 = (portNum/2)*2;
	uint8_t portNum1 = (portNum/2)*2+1;
	
	port_data[0] = portNum0 + (portState << 4);
	port_data[1] = portNum1 + (portState << 4);

	switch (portMode)
	{
		case CURRENT:
			portCurr.port_num = PORTNUM;
			//1. modify struct
			portCurr.port_info[portNum0] = port_data[0];
			portCurr.port_info[portNum1] = port_data[1];
			//2. modify port state
			set_port_state(portNum/2, portState);
			break;
		
		case POWEROFF:
			portPowerOff.port_num = PORTNUM;
			//1. modify struct
			portPowerOff.port_info[portNum0] = port_data[0];
			portPowerOff.port_info[portNum1] = port_data[1];
			//2. modify eeprom data
			eeprom_flag.poweroff_set = 1;
			break;
		
		case NEXT_PWOERUP:
			portNextPowerUP.port_num = PORTNUM;
			//1. modify struct
			portNextPowerUP.port_info[portNum0] = port_data[0];
			portNextPowerUP.port_info[portNum1] = port_data[1];
			//2. modify eeprom data
			eeprom_flag.next_powerup_set = 1;
			break;
		
		case WATCHDOG:
			portWatchdog.port_num = PORTNUM;
			//1. modify struct
			portWatchdog.port_info[portNum0] = port_data[0];
			portWatchdog.port_info[portNum1] = port_data[1];
			//2. modify data
			eeprom_flag.watchdog_set = 1;
			break;
		
		default: 
			retval = EXIT_FAILURE;
			break;
	}
	
	return retval;
}

int port_clear_port_state(void)
{
	int i = 0;
	
	portCurr.port_num = PORTNUM;
	for ( i = 0; i < PORTNUM; i ++ )
	{
		portCurr.port_info[i] = (CURRENT_DEFAULT << 4) | i;
	}

	return EXIT_SUCCESS;
}

int port_clear_watchdog_state(void)
{
	int i = 0;
	portWatchdog.port_num = PORTNUM;
	for ( i = 0; i < PORTNUM; i ++ )
	{
		portWatchdog.port_info[i] = (WATCHDOG_DEFAULT << 4) | i;
	}
	eeprom_flag.watchdog_clear = 1;

	return EXIT_SUCCESS;
}

int set_all_port_state(uint8_t portMode)
{
	int retval = EXIT_SUCCESS;
	uint32_t state = 0;
	int i = 0;
	
	switch (portMode)
	{
		case CURRENT:
			break;
		case POWEROFF:
			for (i=0;i<PORTNUM/2;i++)
			{
				state = portPowerOff.port_info[i*2];
				state = state >> 4;
				set_port_state(i, state);
				port_set_port_state(CURRENT, i*2, state);
			}
			break;
		case NEXT_PWOERUP:
			for (i=0;i<PORTNUM/2;i++)
			{
				state = portNextPowerUP.port_info[i*2];
				state = state >> 4;
				set_port_state(i, state);
				port_set_port_state(CURRENT, i*2, state);
			}
			break;
		case WATCHDOG:
			for (i=0;i<PORTNUM/2;i++)
			{
				state = portWatchdog.port_info[i*2];
				state = state >> 4;
				set_port_state(i, state);
				port_set_port_state(CURRENT, i*2, state);
			}
			break;
		default:
			retval = EXIT_FAILURE;
			break;		
	}

	return retval;
}
