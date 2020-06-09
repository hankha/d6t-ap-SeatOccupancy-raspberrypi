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

#define N_ROW 1
#define N_PIXEL 1
#define N_READ ((N_PIXEL + 1) * 2 + 1)

#define SAMPLE_TIME_0009MS	9
#define SAMPLE_TIME_0010MS	10
#define SAMPLE_TIME_0012MS	12
#define SAMPLE_TIME_0015MS	15
#define SAMPLE_TIME_0020MS	20
#define SAMPLE_TIME_0040MS	40
#define SAMPLE_TIME_0060MS	60
#define SAMPLE_TIME_0100MS	100
#define SAMPLE_TIME_0200MS	200
#define SAMPLE_TIME_0400MS	400
#define SAMPLE_TIME_0800MS	800
#define SAMPLE_TIME_1600MS	1600
#define SAMPLE_TIME_3200MS	3200

#define PARA_0009MS_1	((uint8_t)0x90)
#define PARA_0009MS_2	((uint8_t)0xD3)
#define PARA_0009MS_3	((uint8_t)0x29)
#define PARA_0010MS_1	((uint8_t)0x90)
#define PARA_0010MS_2	((uint8_t)0xD4)
#define PARA_0010MS_3	((uint8_t)0x3C)
#define PARA_0012MS_1	((uint8_t)0x90)
#define PARA_0012MS_2	((uint8_t)0xD5)
#define PARA_0012MS_3	((uint8_t)0x3B)
#define PARA_0015MS_1	((uint8_t)0x90)
#define PARA_0015MS_2	((uint8_t)0xD6)
#define PARA_0015MS_3	((uint8_t)0x32)
#define PARA_0020MS_1	((uint8_t)0x90)
#define PARA_0020MS_2	((uint8_t)0xD7)
#define PARA_0020MS_3	((uint8_t)0x35)
#define PARA_0040MS_1	((uint8_t)0x90)
#define PARA_0040MS_2	((uint8_t)0xD8)
#define PARA_0040MS_3	((uint8_t)0x18)
#define PARA_0060MS_1	((uint8_t)0x90)
#define PARA_0060MS_2	((uint8_t)0xD9)
#define PARA_0060MS_3	((uint8_t)0x1F)
#define PARA_0100MS_1	((uint8_t)0x90)
#define PARA_0100MS_2	((uint8_t)0xDA)
#define PARA_0100MS_3	((uint8_t)0x16)
#define PARA_0200MS_1	((uint8_t)0x90)
#define PARA_0200MS_2	((uint8_t)0xDB)
#define PARA_0200MS_3	((uint8_t)0x11)
#define PARA_0400MS_1	((uint8_t)0x90)
#define PARA_0400MS_2	((uint8_t)0xDC)
#define PARA_0400MS_3	((uint8_t)0x04)
#define PARA_0800MS_1	((uint8_t)0x90)
#define PARA_0800MS_2	((uint8_t)0xDD)
#define PARA_0800MS_3	((uint8_t)0x03)
#define PARA_1600MS_1	((uint8_t)0x90)
#define PARA_1600MS_2	((uint8_t)0xDE)
#define PARA_1600MS_3	((uint8_t)0x0A)
#define PARA_3200MS_1	((uint8_t)0x90)
#define PARA_3200MS_2	((uint8_t)0xDF)
#define PARA_3200MS_3	((uint8_t)0x0D)

#define RASPBERRY_PI_I2C    "/dev/i2c-1"
#define I2CDEV              RASPBERRY_PI_I2C

/***** Setting Parameter 1 *****/
#define comparingNumInc 16 // x samplingTime ms (range: 1 to 39)  (example) 16 x 100 ms -> 1.6 sec
#define comparingNumDec 16  // x samplingTime ms  (range: 1 to 39) (example) 16 x 100 ms -> 1.6 sec
#define threshHoldInc 10 //  /10 degC   (example) 10 -> 1.0 degC (temperature change > 1.0 degC -> Enable)  
#define threshHoldDec 10 //  /10 degC   (example) 10 -> 1.0 degC (temperature change > 1.0 degC -> Disable)
//bool  enablePix[8] = {true, true, true, true, true, true, true, true};
/****************************/

/***** Setting Parameter 2 *****/
#define samplingTime SAMPLE_TIME_0100MS //ms (Can select only, 9ms, 10ms, 12ms, 15ms, 20ms, 40ms, 60ms, 100ms, 200ms, 400ms, 800ms, 1600ms, 3200ms)
/****************************/

uint8_t rbuf[N_READ];
int16_t pix_data = 0;
int16_t seqData[40] = {0};
bool  occuPix = 0;
bool  occuPixFlag = false;
uint8_t  resultOccupancy = 0;
uint16_t  totalCount = 0;

/** JUDGE_occupancy: judge occupancy*/
bool judge_seatOccupancy(void) { 
  int j = 0; 
  for (j = 0; j < 39; j++){
    seqData[39 - j] = seqData[38 - j];
  }
  seqData[0] = pix_data;            
  if (totalCount <= comparingNumInc){
    totalCount++;
  }
  if (totalCount > comparingNumInc){    
    if (occuPix == false){
      if ((int16_t)(seqData[0] - seqData[comparingNumInc]) >= (int16_t)threshHoldInc){
        occuPix = true;
      }
    }
    else{   //resultOccupancy == true
      if ((int16_t)(seqData[comparingNumDec] - seqData[0]) >= (int16_t)threshHoldDec){
        occuPix = false;
      }
    }
    if (resultOccupancy == 0) {                
        if(occuPix == true){
          resultOccupancy = 1;
        }
    }
    else{
      occuPixFlag = false;
      if (occuPix == true){
        occuPixFlag = true;
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
		case SAMPLE_TIME_0009MS:
			para[0] = PARA_0009MS_1;
			para[1] = PARA_0009MS_2;
			para[2] = PARA_0009MS_3;
			break;
		case SAMPLE_TIME_0010MS:
			para[0] = PARA_0010MS_1;
			para[1] = PARA_0010MS_2;
			para[2] = PARA_0010MS_3;
			break;
		case SAMPLE_TIME_0012MS:
			para[0] = PARA_0012MS_1;
			para[1] = PARA_0012MS_2;
			para[2] = PARA_0012MS_3;
			break;
		case SAMPLE_TIME_0015MS:
			para[0] = PARA_0015MS_1;
			para[1] = PARA_0015MS_2;
			para[2] = PARA_0015MS_3;
			break;
		case SAMPLE_TIME_0020MS:
			para[0] = PARA_0020MS_1;
			para[1] = PARA_0020MS_2;
			para[2] = PARA_0020MS_3;
			break;
		case SAMPLE_TIME_0040MS:
			para[0] = PARA_0040MS_1;
			para[1] = PARA_0040MS_2;
			para[2] = PARA_0040MS_3;
			break;
		case SAMPLE_TIME_0060MS:
			para[0] = PARA_0060MS_1;
			para[1] = PARA_0060MS_2;
			para[2] = PARA_0060MS_3;
			break;
		case SAMPLE_TIME_0100MS:
			para[0] = PARA_0100MS_1;
			para[1] = PARA_0100MS_2;
			para[2] = PARA_0100MS_3;
			break;
		case SAMPLE_TIME_0200MS:
			para[0] = PARA_0200MS_1;
			para[1] = PARA_0200MS_2;
			para[2] = PARA_0200MS_3;
			break;
		case SAMPLE_TIME_0400MS:
			para[0] = PARA_0400MS_1;
			para[1] = PARA_0400MS_2;
			para[2] = PARA_0400MS_3;
			break;
		case SAMPLE_TIME_0800MS:
			para[0] = PARA_0800MS_1;
			para[1] = PARA_0800MS_2;
			para[2] = PARA_0800MS_3;
			break;
		case SAMPLE_TIME_1600MS:
			para[0] = PARA_1600MS_1;
			para[1] = PARA_1600MS_2;
			para[2] = PARA_1600MS_3;
			break;
		case SAMPLE_TIME_3200MS:
			para[0] = PARA_3200MS_1;
			para[1] = PARA_3200MS_2;
			para[2] = PARA_3200MS_3;
			break;
		default:
			para[0] = PARA_0100MS_1;
			para[1] = PARA_0100MS_2;
			para[2] = PARA_0100MS_3;
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
	initialSetting();
	while(1){ //add
		memset(rbuf, 0, N_READ);
		uint32_t ret = i2c_read_reg8(D6T_ADDR, D6T_CMD, rbuf, N_READ);
		if (ret) {
		//	return ret;
		}

		if (D6T_checkPEC(rbuf, N_READ - 1)) {
		//	return 2;
		}

		// 1st data is PTAT measurement (: Proportional To Absolute Temperature)
		int16_t itemp = conv8us_s16_le(rbuf, 0);
		printf("PTAT: %4.1f [degC], Temperature: ", itemp / 10.0);
	
		// loop temperature pixels of each thrmopiles measurements
		for (i = 0, j = 2; i < N_PIXEL; i++, j += 2) {
			itemp = conv8us_s16_le(rbuf, j);
			pix_data = itemp; //add
			printf("%4.1f", itemp / 10.0);  // print PTAT & Temperature
			if ((i % N_ROW) == N_ROW - 1) {
				printf(" [degC]");  // wrap text at ROW end.
			} else {
				printf(",");   // print delimiter
			}
		}
		judge_seatOccupancy(); //add
		printf(", Occupancy: %d\n", resultOccupancy);  //add
		delay(samplingTime);  //add
		//return 0;
	}
}
// vi: ft=c:fdm=marker:et:sw=4:tw=80
