/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


/*************************************************
 *  Defines
**************************************************/

//for easy removal of serial debugging output
#define DEBUG
#ifdef DEBUG
    #define debugPrint printf
#else
    #define debugPrint(x)



/*************************************************
 *  Includes
**************************************************/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_task_wdt.h"

#include "driver/uart.h"

#include "config.h"

// Support for GNSS
#include "GPS\GNSS.h"



/*************************************************
 *  Main
**************************************************/

void app_main()
{
    // Configure UART0
    int uart_rx_buffer_size = 2048;
    int uart_tx_buffer_size = 2048;
    uint8_t data[uart_rx_buffer_size];
    bool newPacket = false;
    int bytesRead = 0;
    struct GPS_Data GPS_Data;
    
    
    GPS_init(GNSS_UART_NUM, uart_rx_buffer_size, uart_tx_buffer_size);
    
    debugPrint("Begin Listening for packets...\n");
    while (true) 
    {
        get_GPS_data(data, &bytesRead, &newPacket, uart_num, uart_rx_buffer_size);
        if (newPacket) {        // New Data Packet
            newPacket = false;
        
        /*
            for (int i = 0, j = 0; i < bytesRead; i++) 
            {
                printf("%c", data[i]);
            }
        */
            int stringIndex = stringSearch(data, bytesRead, "GNRMC", 5);    // Search for GNRMC in buffer
            if (stringIndex != -1) 
            {
                /* Parse Data */

                // Reset GPS Data struct
                 GPS_Data.latitude = 0; GPS_Data.longitude = 0;
                 GPS_Data.year = 0; GPS_Data.month = 0; GPS_Data.day = 0;
                 GPS_Data.hour = 0; GPS_Data.minute = 0; GPS_Data.second = 0;  

                int nextIndex = stringIndex + 5 + 1; // stringIndex + length + comma      

                parse_UTC_time(&nextIndex, &GPS_Data, data, bytesRead);  // Parse UTC Time
                nextIndex++;                                             // skip comma
                while (data[nextIndex] != ',')    nextIndex++;           // Skip unwanted data
                nextIndex++;                                             // skip comma

                parse_latitude(&nextIndex, &GPS_Data, data, bytesRead);  // Parse Latitude: ddmm.mmmm 
                nextIndex++;                                             // skip comma         
                while (data[nextIndex] != ',')    nextIndex++;           // Skip unwanted data
                nextIndex++;                                             // skip comma

                parse_longitude(&nextIndex, &GPS_Data, data, bytesRead); // Parse Longitude: dddmm.mmmm
                
                nextIndex++;                                             // skip comma
                while (data[nextIndex] != ',')    nextIndex++;           // Skip unwanted data
                nextIndex++;                                             // skip comma
                while (data[nextIndex] != ',')    nextIndex++;           // Skip unwanted data
                nextIndex++;                                             // skip comma
                while (data[nextIndex] != ',')    nextIndex++;           // Skip unwanted data
                nextIndex++;                                             // skip comma 

                parse_date(&nextIndex, &GPS_Data, data, bytesRead);      // Parse Date: ddmmyy       
                
                print_data_sample(GPS_Data);   // Print GPS Data to terminal            
            }
        }  
    }

    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}


void GetGNSSData( GNSSData* ){
    // Read data length from UART
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
}