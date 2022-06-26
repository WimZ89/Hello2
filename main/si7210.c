/*
 * SI7210.c
 *
 *  Created on: 5 Oct 2021
 *      Author: w.zaagman
 */
//#include "logger.h"
//#include "ui_i2c.h"
//#include "lpc177x_8x_i2c.h"
//#include "lpc177x_8x_timer.h"
#include "esp_log.h"
//#include
#include "driver/i2c.h"
#include <stdbool.h>
#include "sdkconfig.h"
#include <stdio.h>

#define LOG_HALL "HALL"

#define SI7210_adress 0x32
// registers
#define R_chipID 0xC0
#define R_BH 0xc1
#define R_BL 0xC2
#define R_TEMP_MES 0xC3 // 1=temp

#define R_STATUS1 0xC4
#define RB_1_BURST 0x04 // start 1 burst
#define RB_STOP 0x02
#define RB_SLEEP 0x01

#define R_STATUS2 0xC5
#define RB_AUTOINC 0x01

#define R_BURSTCONFIG 0xCD
#define R_TESTCOIL 0xE4


typedef struct {
	int value;		//filtered value points/2 samples terug
	float fvalue;	// float value
	int *memory;	// sample memory
	int memi;		// index in sample memory
	int *kernel;	// distribution weights
	int points;		// points of filter
	int sum;		// sum of distribution
	int top;		// distribution = top/(bot + x*x)
	int bot;		// looks like gaussion exp()
} gaussian;

gaussian filter1;

void gaussian_kernel(int points, int top, int bot, gaussian *gauss){
	ESP_LOGI(LOG_HALL, "Calculating for %d points %d over %d\n",points, top,bot);
	int i,x,y,sumy=0;
	if (gauss->points != points){ // changing parameters.
		if (gauss->memory != 0){
			free(gauss->memory);// remember it does nothing
			free(gauss->kernel);
		}
		gauss->memory= (int*)malloc((points)*sizeof(int));
		gauss->kernel = (int*)malloc((points)*sizeof(int));
		if (gauss->kernel == 0) { // out of heap
			ESP_LOGE(LOG_HALL, "FILTER Heap full");
			gauss->points=0;
			return;
		}
	}
	gauss->memi=0; // restart sampling
	gauss->points=points;
	for (i=0 ; i< points; i++){ // calculate new distribution
		x=i-points/2; // the middle
		y=top/(bot + abs(x*x*x));
		sumy+=y;
		gauss->kernel[i]=y;
	}
	gauss->sum=sumy;
	for (i=0 ; i< gauss->points; i++)
		printf("%d ",gauss->kernel[i]);
	printf("\r\n");
}

bool gaussian_filter(gaussian *gauss, int value){
	int i;
	int sum=0;
	if (gauss->points < 2){ // no filter
		gauss->fvalue=(float)value;
		gauss->value=value;
		return true;
	}
	gauss->memory[gauss->memi++] = value;
	if (gauss->memi >=  gauss->points) { // enough data to filter
		for ( i=0 ; i < gauss->points ; i++){
			sum+=gauss->kernel[i]*gauss->memory[i];
			// TODO overflow using sign change
			// TODO better.. ofset to no overflow range.
			if (i>0) // move back memory
				gauss->memory[i-1]=gauss->memory[i];
		}
		gauss->memi--;
		gauss->fvalue=(float)sum/(float)gauss->sum;
		gauss->value=sum/gauss->sum;
		return true;
	}
	return false;
}

void gauss_update(int points, int top, int bot){
	gaussian_kernel(points, top, bot, &filter1);
}
void gauss_flush(gaussian *gauss){
	// flush old data in memory
	gauss->memi=0;
}


#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

void SetClock (bool highspeed)
{	int high,low;
	i2c_get_period(I2C_NUM_0,&high,&low);
	ESP_LOGI(LOG_HALL, "i2c periods %d %d",high,low);
	if (highspeed){
		high=22;
		low=25; // 835kHz
	} else {
		high=44;
		low=50; // 380kHz
	}
	i2c_set_period(I2C_NUM_0,high,low);
}


void ui_i2c_startTransferData(uint8_t address, uint8_t *data_wr, size_t size_wr,
		uint8_t *data_rd, size_t size_rd){
	ESP_ERROR_CHECK_WITHOUT_ABORT(
		i2c_master_write_read_device(I2C_NUM_0, address,
			data_wr, size_wr,
			data_rd, size_rd,
			1000 / portTICK_RATE_MS));
}

void ui_i2c_writeData(uint8_t address, uint8_t *data_wr, size_t size_wr){
	ESP_ERROR_CHECK_WITHOUT_ABORT(
	i2c_master_write_to_device(I2C_NUM_0, address,
			data_wr, size_wr,
			1000 / portTICK_RATE_MS)
	);
}

int SI7210_init(void){
	uint8_t txdata[3];
	uint8_t rxdata[10];
	uint8_t reg;


	txdata[0]=R_STATUS2; //
	txdata[1]=RB_AUTOINC; //
	ui_i2c_writeData(SI7210_adress, txdata, 2);

	txdata[0]=R_STATUS1; //
	txdata[1]=RB_1_BURST; //
	ui_i2c_writeData(SI7210_adress, txdata, 2);

	// bij 5 helft niet fresh
#define BURST 3
	uint8_t iir_burstsize=BURST;
	uint8_t fir_burstsize=BURST;
	uint8_t burst=iir_burstsize<<1;
	burst |=fir_burstsize<<5;
	// df_iir = 0 is fir. dus fir_burstsize
//	burst |=1;
	txdata[0]=R_BURSTCONFIG; //
	txdata[1]=burst; //
	ui_i2c_writeData(SI7210_adress, txdata, 2);

	for ( reg = R_chipID ; reg <= R_BURSTCONFIG ; reg++) {
		txdata[0]=reg; //
		ui_i2c_startTransferData(SI7210_adress, txdata, 1, rxdata, 1);
		printf("SI7210 %x %x\r\n", reg, rxdata[0]);
	}

	gaussian_kernel(25,300,3, &filter1);
	SetClock(true); // high speed.. could be problem for others
	return 0;
}


float HAL_data[200];
char floatbuf[20]; // debug_print geen float // TODO gebruik vsprintf in debugprintf
uint8_t txdata[10];
uint8_t rxdata[10];

int SI7210_read(void){
	int gz;
	txdata[0]=R_STATUS1; //
	txdata[1]=RB_1_BURST; // start meting
	ui_i2c_writeData(SI7210_adress, txdata, 2);
	txdata[0]=R_BH; // lees RBH
	ui_i2c_startTransferData(SI7210_adress, txdata, 1, rxdata, 2);
	gz=0;
	gz |= (rxdata[0] << 8) & 0xff00;
	gz |= rxdata[1]  & 0xff;
	return gz;
}

void data_uart_send_string(char *s);

int SI7210_read_array(int count){
	static int last_gz,glitchcount;
	int gz=0,i,halidx=0,glitch_value=0;
	int not_fresh=0;
	count=count%200;
	gauss_flush(&filter1);
	glitchcount=0;
	for(i=0 ; i < count; i++){
		gz=SI7210_read();
		if ((gz & 0x8000) == 0)
			not_fresh++;
		gz &= 0x7fff;
		gz -=16384;// 0 Tesla. in theorie. praktijk heeft ofset.

		// remove glitches
		if(i>0 && abs(last_gz - gz) > 1000){ // komen niet van not fresh. EMP:)
			glitch_value=gz; // meestal 1
			gz=last_gz;
			glitchcount++;
		}
		last_gz=gz;

		if (halidx < 200)
			if ( gaussian_filter(&filter1,gz) ){
				HAL_data[halidx++]=filter1.fvalue;
			}
	}
	data_uart_send_string("HALL");
	for(i=0 ; i < halidx; i++){
		sprintf(floatbuf," %f",HAL_data[i]);
		data_uart_send_string(floatbuf);
	}
	data_uart_send_string("\r\n");
	data_uart_send_string("HALL PLOT\r\n");
	printf("BZ %x %x %d\r\n",rxdata[0], rxdata[1], gz);
	if (not_fresh > 0)
		printf("Not Fresh %d\r\n",not_fresh);
	if (glitchcount > 0)
		printf("Glitches %d %x\r\n",glitchcount,glitch_value);

	return gz;
}

int SI7210_read_array_dummy(int count){
	int i;
	data_uart_send_string("HALL");
	for(i=0 ; i < count; i++){
		sprintf(floatbuf," %f",i*.01);
		data_uart_send_string(floatbuf);
	}
	data_uart_send_string("\r\n");
	data_uart_send_string("HALL PLOT\r\n");
	return 0;
}

