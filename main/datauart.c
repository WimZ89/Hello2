/* UART for communicating with python script.
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "string.h"


#define TAG  "UART_console"

#define BUF_SIZE (1024*8)
#define SIZEOFLINEBUFFER 800 // for SWNP. 208B
char linebuffer[SIZEOFLINEBUFFER];
int  linebufferi;
int  linebuffer_chunks;

uint8_t linedata[500];
//____________________________________________________________________________ Hex and bytes
int nibble(int c){
	if (c < 'A')
		return c-'0';
	if (c > 'F') // uppercase?
		return c-'a'+ 10;
	else
		return c-'A'+ 10;
}

int hex2bytes(void * data, void * destination, int maxsize) {
	int size=0;
	char * pos= (char*) data;
	char val,*dest=(char*)destination;
	while(*pos != 0) {
		while ((*pos) == ' ') // skip spaces
			pos++;
		size++;
		if (size > maxsize)
			return 0;
		val  = nibble(*pos)<<4;pos++;
		val |= nibble(*pos);pos++;
		*dest=val;
		dest++;
	}
	return size;
}
int bytes2hex(void * data, void * destination, int size) {
	uint8_t *d=data;
	char *s=(char*) destination;
	*s=0; // size can be 0
	for (int i=0; i< size; i++){
		s+=sprintf(s,"%02x",d[i]);
	}
	return 1;
}

//____________________________________________________________________________ LINE RX
void gauss_update(int points, int top, int bot);
int SI7210_read_array(int count);
int SI7210_read_array_dummy(int count);
//int SI7210_read(void);

//void MCP3421_init(int gain);
//float MCP3421_value(void);
//float MCP3426_value(void);
//extern double adc_val[2];
void gotline(char * line){
//	int size;
//	ESP_LOGI(TAG, "Got line (%d,%d) '%s'\n",linebuffer_chunks,strlen(line),line);
	switch (line[0]){
	case '\r': // Scan
//		MCP3426_value();
//		ESP_LOGI(TAG,"ADC %f %f",adc_val[0],adc_val[1]);
		break;
	case 'S': // Scan
		SI7210_read_array(180);
		break;
	case 's': // Scan
		SI7210_read_array_dummy(180);
		break;
	case 'a': // Scan
		break;
	case 'L': {
		switch (line[1]){
		case 'G': {// new filter
			int points,  top,  bot;
			sscanf(line+2,"%d%d%d",&points,  &top,  &bot);
			gauss_update( points,  top,  bot);
		}
		break;
		default:
			ESP_LOGE(TAG, "Unknown command after L %s",line);
			break;
		}
	}
	break;
	default:
		ESP_LOGE(TAG, "Unknown command %s",line);
		break;

	}
}

void addlinechunk(uint8_t * buffer,int length){
	char *nl=0;
	//	printhex("linechunk", buffer, length);
	//	printhex("line", linebuffer, 10);
	if (length == 0) return;
	if ((linebufferi + length + 1) > SIZEOFLINEBUFFER){
		ESP_LOGE(TAG, "linebuffer overflow %d", linebufferi + length + 1 );
	}
	memcpy(linebuffer+linebufferi,buffer,length);
	//					*(linebuffer+linebufferi+1)=0; init altijd 0
	linebuffer_chunks++;
	if ((nl=strchr(linebuffer+linebufferi,'\n')) != 0){
		do {
			*nl=0;
			gotline(linebuffer);
			memcpy(linebuffer,nl+1+1,nl-linebuffer+SIZEOFLINEBUFFER); // can ther be non zeros?
			linebufferi = 0;
		} while ((nl=strchr(linebuffer+linebufferi,'\n')) != 0);
		memset(linebuffer,0,SIZEOFLINEBUFFER);
		linebuffer_chunks=0;
		linebufferi=0;
		return;
	}
	linebufferi+=length;

}

void data_uart_send(const void *src, size_t size){
	uart_write_bytes(UART_NUM_1, src, size);
}

void data_uart_send_string(char *s){
	uart_write_bytes(UART_NUM_1, s, strlen(s));
}

void data_uart_task(void *arg)
{
	/* Configure parameters of an UART driver,
	 * communication pins and install the driver */
	uart_config_t uart_config = {
			.baud_rate = 115200,
			.data_bits = UART_DATA_8_BITS,
			.parity    = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
			.source_clk = UART_SCLK_APB,
	};
	int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
	intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
	ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
	ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, 2, 3, -1, -1)); //tx rx ..

	// Configure a temporary buffer for the incoming data
	uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
	printf("uart_echo_task started");
	data_uart_send_string("Started data uart\r\n");
	while (1) {
		// Read data from the UART
		int len = uart_read_bytes(UART_NUM_1, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
		addlinechunk(data,len);
		// Write data back to the UART
//		uart_write_bytes(UART_NUM_1, (const char *) data, len);
		if (len) {
			data[len] = '\0';
			ESP_LOGI(TAG, "Recv str: %s", (char *) data);
		}
	}
}

