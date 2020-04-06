/*
 * MIT License
 * Copyright (c) 2019, 2018 - present OMRON Corporation
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

/* includes */
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdbool.h>
#include <time.h>

/* defines */
#define D6T_ADDR 0x0A  // for I2C 7bit address
#define D6T_CMD 0x4C  // for D6T-44L-06/06H, D6T-8L-09/09H, for D6T-1A-01/02

#define N_ROW 8
#define N_PIXEL 8
#define N_READ ((N_PIXEL + 1) * 2 + 1)

#define SAMPLE_TIME_0050MS	50
#define SAMPLE_TIME_0070MS	70
#define SAMPLE_TIME_0140MS	140
#define SAMPLE_TIME_0250MS	250
#define SAMPLE_TIME_0500MS	500
#define SAMPLE_TIME_1000MS	1000
#define SAMPLE_TIME_2000MS	2000

#define PARA_0050MS_1	((uint8_t)0x80)
#define PARA_0050MS_2	((uint8_t)0x37)
#define PARA_0050MS_3	((uint8_t)0xCC)
#define PARA_0070MS_1	((uint8_t)0x80)
#define PARA_0070MS_2	((uint8_t)0x38)
#define PARA_0070MS_3	((uint8_t)0xE1)
#define PARA_0140MS_1	((uint8_t)0x80)
#define PARA_0140MS_2	((uint8_t)0x39)
#define PARA_0140MS_3	((uint8_t)0xE6)
#define PARA_0250MS_1	((uint8_t)0x80)
#define PARA_0250MS_2	((uint8_t)0x3A)
#define PARA_0250MS_3	((uint8_t)0xEF)
#define PARA_0500MS_1	((uint8_t)0x80)
#define PARA_0500MS_2	((uint8_t)0x3B)
#define PARA_0500MS_3	((uint8_t)0xE8)
#define PARA_1000MS_1	((uint8_t)0x80)
#define PARA_1000MS_2	((uint8_t)0x3C)
#define PARA_1000MS_3	((uint8_t)0xFD)
#define PARA_2000MS_1	((uint8_t)0x80)
#define PARA_2000MS_2	((uint8_t)0x3D)
#define PARA_2000MS_3	((uint8_t)0xFA)

#define RASPBERRY_PI_I2C    "/dev/i2c-1"
#define I2CDEV              RASPBERRY_PI_I2C

/***** Setting Parameter *****/
#define comparingNumInc 6  // x samplingTime ms   (example) 6 x 250 ms -> 1.5 sec
#define comparingNumDec 6  // x samplingTime ms   (example) 6 x 250 ms -> 1.5 sec
#define threshHoldInc 10 //  /10 degC   (example) 10 -> 1.0 degC (temperature change > 1.0 degC -> Enable) 
#define threshHoldDec 10 //  /10 degC   (example) 10 -> 1.0 degC (temperature change > 1.0 degC -> Enable) 
bool  enablePix[8] = {true, true, true, true, true, true, true, true};
/****************************/

/***** Setting Parameter 2 *****/
#define samplingTime SAMPLE_TIME_0250MS //ms (Can select only, 50ms, 70ms, 140ms, 250ms, 500ms, 1000ms, 2000ms)
/****************************/

uint8_t rbuf[N_READ];
int16_t pix_data[8] = {0};
int16_t seqData[8][40] = {0};
bool  occuPix[8] = {0};
bool  occuPixFlag = false;
uint8_t  resultOccupancy = 0;
uint16_t  totalCount = 0;

/** JUDGE_occupancy: judge occupancy*/
bool judge_seatOccupancy(void) { 
  int i = 0;
  int j = 0; 
  for (i = 0; i < 8; i++){
    for (j = 0; j < 39; j++){
      seqData[i][39 - j] = seqData[i][38 - j];
    }
    seqData[i][0] = pix_data[i];            
  }
  if (totalCount <= comparingNumInc){
    totalCount++;
  }
  if (totalCount > comparingNumInc){
    for (i = 0; i < 8; i++){
      if (enablePix[i] == true){
        if (occuPix[i] == false){
           if ((int16_t)(seqData[i][0] - seqData[i][comparingNumInc]) > (int16_t)threshHoldInc){
            occuPix[i] = true;
          }
        }
        else{   
		  if ((int16_t)(seqData[i][comparingNumDec] - seqData[i][0]) > (int16_t)threshHoldDec){
			occuPix[i] = false;
          }
        }
      }
    }
    if (resultOccupancy == 0) {
      for (i = 0; i < 8; i++){                   
        if(occuPix[i] == true){
          resultOccupancy = 1;
          break;
        }
      }
    }
    else{  //resultOccupancy == true
      occuPixFlag = false;
      for (i = 0; i < 8; i++){
        if (occuPix[i] == true){
          occuPixFlag = true;
          break;
        }
        else{                            
        }
      }
      if (occuPixFlag == false){
        resultOccupancy = 0;
      }
    }
  }
  return true;
}


/* I2C functions */
/** <!-- i2c_read_reg8 {{{1 --> I2C read function for bytes transfer.
 */
uint32_t i2c_read_reg8(uint8_t devAddr, uint8_t regAddr,
                       uint8_t *data, int length
) {
    int fd = open(I2CDEV, O_RDWR);

    if (fd < 0) {
        fprintf(stderr, "Failed to open device: %s\n", strerror(errno));
        return 21;
    }
    int err = 0;
    do {
        if (ioctl(fd, I2C_SLAVE, devAddr) < 0) {
            fprintf(stderr, "Failed to select device: %s\n", strerror(errno));
            err = 22; break;
        }
        if (write(fd, &regAddr, 1) != 1) {
            fprintf(stderr, "Failed to write reg: %s\n", strerror(errno));
            err = 23; break;
        }
        int count = read(fd, data, length);
        if (count < 0) {
            fprintf(stderr, "Failed to read device(%d): %s\n",
                    count, strerror(errno));
            err = 24; break;
        } else if (count != length) {
            fprintf(stderr, "Short read  from device, expected %d, got %d\n",
                    length, count);
            err = 25; break;
        }
    } while (false);
    close(fd);
    return err;
}

/** <!-- i2c_write_reg8 {{{1 --> I2C read function for bytes transfer.
 */
uint32_t i2c_write_reg8(uint8_t devAddr,
                        uint8_t *data, int length
) {
    int fd = open(I2CDEV, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Failed to open device: %s\n", strerror(errno));
        return 21;
    }
    int err = 0;
    do {
        if (ioctl(fd, I2C_SLAVE, devAddr) < 0) {
            fprintf(stderr, "Failed to select device: %s\n", strerror(errno));
            err = 22; break;
        }
        if (write(fd, data, length) != length) {
            fprintf(stderr, "Failed to write reg: %s\n", strerror(errno));
            err = 23; break;
        }
    } while (false);
    close(fd);
    return err;
}

uint8_t calc_crc(uint8_t data) {
    int index;
    uint8_t temp;
    for (index = 0; index < 8; index++) {
        temp = data;
        data <<= 1;
        if (temp & 0x80) {data ^= 0x07;}
    }
    return data;
}

/** <!-- D6T_checkPEC {{{ 1--> D6T PEC(Packet Error Check) calculation.
 * calculate the data sequence,
 * from an I2C Read client address (8bit) to thermal data end.
 */
bool D6T_checkPEC(uint8_t buf[], int n) {
    int i;
    uint8_t crc = calc_crc((D6T_ADDR << 1) | 1);  // I2C Read address (8bit)
    for (i = 0; i < n; i++) {
        crc = calc_crc(buf[i] ^ crc);
    }
    bool ret = crc != buf[n];
    if (ret) {
        fprintf(stderr,
                "PEC check failed: %02X(cal)-%02X(get)\n", crc, buf[n]);
    }
    return ret;
}


/** <!-- conv8us_s16_le {{{1 --> convert a 16bit data from the byte stream.
 */
int16_t conv8us_s16_le(uint8_t* buf, int n) {
    int ret;
    ret = buf[n];
    ret += buf[n + 1] << 8;
    return (int16_t)ret;   // and convert negative.
}


void delay(int msec) {
    struct timespec ts = {.tv_sec = msec / 1000,
                          .tv_nsec = (msec % 1000) * 1000000};
    nanosleep(&ts, NULL);
}

void initialSetting(void) {

	uint8_t para[3] = {0};
	switch(samplingTime){
		case SAMPLE_TIME_0050MS:
			para[0] = PARA_0050MS_1;
			para[1] = PARA_0050MS_2;
			para[2] = PARA_0050MS_3;
			break;
		case SAMPLE_TIME_0070MS:
			para[0] = PARA_0070MS_1;
			para[1] = PARA_0070MS_2;
			para[2] = PARA_0070MS_3;
			break;
		case SAMPLE_TIME_0140MS:
			para[0] = PARA_0140MS_1;
			para[1] = PARA_0140MS_2;
			para[2] = PARA_0140MS_3;
			break;
		case SAMPLE_TIME_0250MS:
			para[0] = PARA_0250MS_1;
			para[1] = PARA_0250MS_2;
			para[2] = PARA_0250MS_3;
			break;
		case SAMPLE_TIME_0500MS:
			para[0] = PARA_0500MS_1;
			para[1] = PARA_0500MS_2;
			para[2] = PARA_0500MS_3;
			break;
		case SAMPLE_TIME_1000MS:
			para[0] = PARA_1000MS_1;
			para[1] = PARA_1000MS_2;
			para[2] = PARA_1000MS_3;
			break;
		case SAMPLE_TIME_2000MS:
			para[0] = PARA_2000MS_1;
			para[1] = PARA_2000MS_2;
			para[2] = PARA_2000MS_3;
			break;
		default:
			para[0] = PARA_0250MS_1;
			para[1] = PARA_0250MS_2;
			para[2] = PARA_0250MS_3;
			break;
	}
	
	uint8_t dat1[] = {0x02, 0x00, 0x01, 0xee};
    i2c_write_reg8(D6T_ADDR, dat1, sizeof(dat1));
    uint8_t dat2[] = {0x05, para[0], para[1], para[2]};
    i2c_write_reg8(D6T_ADDR, dat2, sizeof(dat2));
    uint8_t dat3[] = {0x03, 0x00, 0x03, 0x8b};
    i2c_write_reg8(D6T_ADDR, dat3, sizeof(dat3));
    uint8_t dat4[] = {0x03, 0x00, 0x07, 0x97};
    i2c_write_reg8(D6T_ADDR, dat4, sizeof(dat4));
    uint8_t dat5[] = {0x02, 0x00, 0x00, 0xe9};
    i2c_write_reg8(D6T_ADDR, dat5, sizeof(dat5));
    delay(2*samplingTime);

}

/** <!-- main - Thermal sensor {{{1 -->
 * 1. read sensor.
 * 2. output results, format is: [degC]
 */
int main() {
    int i, j;

	while(1){ //add
		memset(rbuf, 0, N_READ);
		uint32_t ret = i2c_read_reg8(D6T_ADDR, D6T_CMD, rbuf, N_READ);
		if (ret) {
			return ret;
		}

		if (D6T_checkPEC(rbuf, N_READ - 1)) {
			return 2;
		}

		// 1st data is PTAT measurement (: Proportional To Absolute Temperature)
		int16_t itemp = conv8us_s16_le(rbuf, 0);
		printf("PTAT: %6.1f[degC]", itemp / 10.0);     //change

		// loop temperature pixels of each thrmopiles measurements
		for (i = 0, j = 2; i < N_PIXEL; i++, j += 2) {
			itemp = conv8us_s16_le(rbuf, j);
			pix_data[i] = itemp; //add
			printf("%4.1f", itemp / 10.0);  // print PTAT & Temperature
			if ((i % N_ROW) == N_ROW - 1) {
				printf(" [degC]");  // wrap text at ROW end. //change
			} else {
				printf(",");   // print delimiter
			}
		}
		judge_seatOccupancy(); //add
		printf("Occupancy: %6.1f\n", resultOccupancy);  //add
		delay(samplingTime);  //add
		return 0;
	}	//add
}
// vi: ft=c:fdm=marker:et:sw=4:tw=80
