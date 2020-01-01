/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_task_wdt.h"

#include "driver/uart.h"


void app_main()
{
    printf("Hello world!\n");
    int err = 0;

    // Configure UART0
    const int uart_num = UART_NUM_1;
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };

    // Configure UART parameters
    err = uart_param_config(uart_num, &uart_config);
    printf("param_config returned with code: %d\n", err);

    // Set UART pins(Tx, Rx, RTS, CTS)
    err = uart_set_pin(uart_num, 
                        22,     // TX = IO 22 
                        21,     // RX = IO 21
                        UART_PIN_NO_CHANGE, // RTS not used
                        UART_PIN_NO_CHANGE);// CTS not used
    printf("set_pin returned with code %d\n", err);

    //install new driver
    int uart_rx_buffer_size = 2048;
    int uart_tx_buffer_size = 2048;
    QueueHandle_t uart_queue;
    uart_driver_install(uart_num, uart_rx_buffer_size, \
                        uart_tx_buffer_size, 10, &uart_queue, 0);

    unsigned int prevLength = 0;
    unsigned int newLength = 0;
    bool newPacket = false;
    printf("Begin Listening for packets...");
    while (true) {
        //reset WDT
        esp_task_wdt_reset();

        // Read data length from UART.
        uint8_t data[uart_rx_buffer_size];
        prevLength = newLength;
        uart_get_buffered_data_len(uart_num, (size_t*)&newLength);
        
        //it is still sending
        if(newLength != prevLength){
            //delay for .1 second
            printf("\nReceiving... %d", newLength);
            vTaskDelay(200 / portTICK_PERIOD_MS);
            newPacket = true;
        }

        //otherwise it must have stopped sending
        else if(newPacket){
            //get the buffer contents
            int bytesRead = uart_read_bytes(uart_num, data, newLength, 100);
            printf("\nUART%d received %d bytes:\n", uart_num, bytesRead);
            //print the buffer contents
            for(int i = 0; i < bytesRead; i++){
                printf("%c",(unsigned int)data[i]);
            }

            //packet has been printed. Reset the system.
            newPacket = false;
        }
    }

    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
