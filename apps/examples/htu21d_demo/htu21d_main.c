/****************************************************************************
 * apps/system/i2c/i2c_main.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdarg.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>

#define MAX_RANGE (65536.0)
 
/****************************************************************************
 * Name: i2cdev_transfer
 ****************************************************************************/

static int i2cdev_transfer(int fd, FAR struct i2c_msg_s *msgv, int msgc)
{
	struct i2c_transfer_s xfer;

	/* Set up the IOCTL argument */

	xfer.msgv = msgv;
	xfer.msgc = msgc;

	/* Perform the IOCTL */

	return ioctl(fd, I2CIOC_TRANSFER, (unsigned long)((uintptr_t)&xfer));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
	/*---------------------------------------------------------------------------*/
#define CRC8_POLY 0x31 //Polynomial 0x31(X8 + X5 + X4 + 1)
	uint8_t CRC8(uint8_t crc, uint8_t byte)
	{
		uint8_t i;
	 
		crc ^= byte;
		for(i = 0; i < 8; i++)
		{
			if(crc & 0x80)
			{
				crc = (crc<< 1)^CRC8_POLY;
			}
			else
			{
				crc = crc<< 1;
			}
		}
	 
		return crc;
	}
	 

/****************************************************************************
 * Name: htu21d_main
 ****************************************************************************/

#ifdef BUILD_MODULE
int main(int argc, FAR char *argv[])
#else
int htu21d_main(int argc, char *argv[])
#endif
{
  struct i2c_msg_s msg[2];
  FAR char *ptr;
  union
  {
    uint16_t data16;
    uint8_t  data8;
  } u;
	int fd;
	int rc;
  uint8_t regaddr;
	uint8_t buf[4] = {0};
  uint8_t addr = 0x40;
	uint8_t acc;
	int a,b;
	int tmp;
	float Temp;
	float RH;

  /* A register address other than zero may be provided on the command line.
   * However, for backward compatibility, the address zero will be used
   * unless the address is specifically included on the command line for
   * this command.
   */

	printf("Hello, This is htu21d demo.\r\n");
   

  /* There should be exactly two more things on the command line:  The first and
   * last addresses to be probed.
   */

  /* Get a handle to the I2C bus */

  fd = open("/dev/i2c1",O_RDWR);
  if (fd < 0)
  {
     printf("Failed to get bus %d\n", 1);
     goto errout;
  }

	/* Set up data structures */
	
	regaddr 				 = 0xe3;
	
	msg[0].frequency = 100000;
	msg[0].addr 		 = addr;
	msg[0].flags		 = I2C_M_NOSTOP;
	msg[0].buffer 	 = &regaddr;
	msg[0].length 	 = 1;
	
	msg[1].frequency = 100000;
	msg[1].addr 		 = addr;
	msg[1].flags		 = I2C_M_READ;
	msg[1].buffer 	 = buf;
	msg[1].length 	 = 3;
	
	rc = i2cdev_transfer(fd, &msg[0], 1);
	if (rc == OK)
	{
		rc = i2cdev_transfer(fd, &msg[1], 3);

		acc = CRC8(0,buf[0]);
		acc = CRC8(acc,buf[1]);

		if(acc == buf[2])
		{
			
			tmp = (buf[0]<<8) | (buf[1] & 0xFC);
			Temp = -46.85 + 175.72*(float)tmp/MAX_RANGE;
			a = (int)Temp;
			b = (Temp - a)*100;
			printf("\r\nCurrent Temp : %d.%02d C\r\n",a,b);
		}
		else
		{
			printf("Read : %02x%02x ,crc %02x, acc = %02x\r\n",buf[0],buf[1],buf[2],acc);
		}
	}
	else
	{
		printf("HTU21-D Error: %d\r\n",rc);
	}

	regaddr 				 = 0xe5;
	
	msg[0].frequency = 100000;
	msg[0].addr 		 = addr;
	msg[0].flags		 = I2C_M_NOSTOP;
	msg[0].buffer 	 = &regaddr;
	msg[0].length 	 = 1;
	
	msg[1].frequency = 100000;
	msg[1].addr 		 = addr;
	msg[1].flags		 = I2C_M_READ;
	msg[1].buffer 	 = buf;
	msg[1].length 	 = 3;
	
	rc = i2cdev_transfer(fd, &msg[0], 1);
	if (rc == OK)
	{
		rc = i2cdev_transfer(fd, &msg[1], 3);

		acc = CRC8(0,buf[0]);
		acc = CRC8(acc,buf[1]);


		if(acc == buf[2])
		{
			tmp = (buf[0]<<8) | (buf[1] & 0xFC);
			RH = -6 + 125.0*((float)tmp/MAX_RANGE);
			a = (int)RH;
			b = (RH - a)*100;
			printf("\r\nCurrent RH : %d.%02d \r\n",a,b);
		}
		else
		{
			printf("Read : %02x%02x ,crc %02x, acc = %02x\r\n",buf[0],buf[1],buf[2],acc);
		}
	}
	else
	{
		printf("HTU21-D Error: %d\r\n",rc);
	}
	

  (void)close(fd);

errout:

  /* Restore the previous "sticky" register address unless a new register
   * address was provided on the command line.  In that case the new
   * register address is retained.
   */

  return OK;
}

