/* 
 * This code is provided "as is" and without any warranty. This file now can be joined to libopencm3
 * and follow any legal disclaimer requested by the original project. 
 * 
 * coded by manutronics 
 * 
 * The code in function clock_get_cpu_speed() was based in a awesome project from SJSU 
 * (San Jose State Univeristy), the SjOne Board source code.
 * All rights to SocialLedge.com - Copyright (C) 2013 and a very than kyou to the 
 * Professor Preetpal Kang
 * 
 * I would suggest that anyone that is using this repo get a look into the new project 
 * https://github.com/libembeddedhal/SJSU-Dev2.git
 * This is anextension of the original one. 
 * 
 *     Copy of the copyright header
 *     ---------------------------
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

#include <libopencm3/cm3/common.h>
#include <libopencm3/lpc17xx/clock.h>


uint32_t clock_get_cpu_speed()
{
	uint32_t clock = 0;
	volatile uint32_t p_status = CLK_PLL0STAT;

	if (((CLK_PLL0STAT >> 24) & 3) == 3)
	{
		switch (CLK_CLKSRCSEL & 3)
		{
			case 0:
				clock = IRC_OSC;
				break;

			case 1: 
				clock = MAIN_OSC;
				break;

			case 2: 
				clock = RTC_OSC;
				break;
		}
		/* Formulae from UM10360 section 4.5.10 PLL0 Freq Calculation */
		clock *=  ((2 * ((p_status & 0x7FFF) + 1)))
						/ (((p_status >> 16) & 0xFF) + 1)
						/ ((CLK_CCLKCFG & 0xFF) + 1);
	}
	else
	{
		switch (CLK_CLKSRCSEL & 0x03)
		{
			case 0:
				clock = IRC_OSC / ((CLK_CCLKCFG & 0xFF) + 1);
				break;
			case 1:
				clock = MAIN_OSC / ((CLK_CCLKCFG & 0xFF) + 1);
				break;
			case 2:
				clock = RTC_OSC / ((CLK_CCLKCFG & 0xFF) + 1);
				break;
		}
	}
	return clock;
}

void sysclock_setup(void)
{
	uint32_t pllcon;

	//disconnect and disable from pll0 for configure sequence.
	CLK_PLL0CON = 0;
	CLK_PLL0FEED = 0xAA;
	CLK_PLL0FEED = 0x55;

	//Enable the main oscillator acoording with UM10360 section 3.7.1
	if(!(CLK_SCS & CLK_SCS_OSCSTAT))
	{
		CLK_SCS |= CLK_SCS_OSCEN;
	}
	//Verifies if the main oscillator is stabilized...
	while((CLK_SCS & CLK_SCS_OSCSTAT)!=CLK_SCS_OSCSTAT);

	CLK_CLKSRCSEL = CLK_CLKSRCSEL_MAIN;

	//M multiplier is set to 19 to reach 480MHz PLL out from one 12MHz crystal
	//Pll pos divisor is 4 (CCLKCFG + 1) to achieve 120MHz CPU clock.
	CLK_PLL0CFG = 19;
	CLK_CCLKCFG = 3;
	CLK_PLL0CON = 1;
	CLK_PLL0FEED = 0xAA;
	CLK_PLL0FEED = 0x55;

	pllcon = CLK_PLL0CON;
	pllcon |= CLK_PLLCON_ENABLE;
	CLK_PLL0CON = pllcon;
	CLK_PLL0FEED = 0xAA;
	CLK_PLL0FEED = 0x55;

	// CLK_CCLKCFG = 3;
	while((CLK_PLL0STAT & CLK_PLL0STAT_PLOCK)!=CLK_PLL0STAT_PLOCK);

	CLK_PLL0CON = CLK_PLLCON_CONNECT|CLK_PLLCON_ENABLE;
	CLK_PLL0FEED = 0xAA;
	CLK_PLL0FEED = 0x55;
	//checking if everything went well.
	while((CLK_PLL0STAT & (CLK_PLL0STAT_CONNECT|CLK_PLL0STAT_ENABLE))!=(CLK_PLL0STAT_CONNECT|CLK_PLL0STAT_ENABLE));
}