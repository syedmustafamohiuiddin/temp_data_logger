/**
	@author Thomas Grunenberg
	@author syed mustafa mohiuddin
	@version 0.1
	@file main.c
	@brief Main programm for temperature data logger
*/

const char MtrNum[] __attribute__((__progmem__)) = "26293";

/**
	@brief The CPU speed in Hz
*/
#define F_CPU 8000000UL


#define TEMPSENOR_OFFSET 600 // TODO

/**
	@brief I2C Address of the DS1307
*/
#define DS1307_I2C_ADR 0xd0 //TODO: Enter the Address of the DS1307

/******************************************************************************/
/* INCLUDED FILES                                                             */
/******************************************************************************/
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "i2c_master.h"
#include "init.h"
#include "lcd.h"
#include "stdio.h"
/******************************************************************************/


/******************************************************************************/
/* GLOBAL MEMORY                                                              */
/******************************************************************************/
char* dayofweek[8] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun", "Err"};

// Global Time memory
uint8_t second;
uint8_t minute;
uint8_t hour;
uint8_t day;
uint8_t month;
uint8_t year;
uint8_t weekday;
uint8_t save;               // variable for write address
uint8_t load;               // variable for read address
uint8_t count;              // variable for value count
uint8_t value;
uint8_t temperature;




/******************************************************************************/



/******************************************************************************/
/* FUNCTIONS                                                                  */
/******************************************************************************/

/**
	@brief Convert from BCD to Binary
	@param in BCD input (00-99)
	@return Binary output
*/
uint8_t ds1307_decodeBcd(uint8_t in){
	return ((in >> 4) * 10 + (in & 0xF));//TODI
}
/******************************************************************************/

/**
	@brief Convert from Binary to BCD
	@param in Binary input (0-99)
	@return BCD output
*/
uint8_t ds1307_encodeBcd(uint8_t in){
	return ((in / 10) << 4 ) | (in % 10); 
}
/******************************************************************************/


/**
	@brief Show time/date with the LCD
*/
void display_standby(uint16_t t){
	char str[16];
	
	// Time and Year
	snprintf(str, 16, "%02d:%02d:%02d  20%02d", hour, minute,
			second, year);
	
	lcd_clear();
	lcd_string(str);
	
	
	// Date and Temperature
	snprintf(str, 16, "%02d.%02d  %d.%d C", day, month, t/10, t%10);
	
	lcd_setcursor(0,2);
	lcd_string(str);

	return;
}
/******************************************************************************/

/**
	@brief Write a row byte to the DS1307
	@param adr address to write
	@param data byte to write
*/
void ds1307_write(uint8_t adr, uint8_t data){
	
	if (i2c_master_open_write(DS1307_I2C_ADR))
		return;
	
	i2c_master_write(adr);
	i2c_master_write(data);
	
	i2c_master_close();
}
/******************************************************************************/

/**
	@brief Read a row byte from the DS1307
	@param adr address to read
	@return the received byte
*/
uint8_t ds1307_read(uint8_t adr){
	uint8_t ret;

	if (i2c_master_open_write(DS1307_I2C_ADR))
		return 0;
	
	i2c_master_write(adr);
	i2c_master_open_read(DS1307_I2C_ADR);
	ret = i2c_master_read_last();
	
	i2c_master_close();

	return ret;

}
/******************************************************************************/

/**
	@brief Start or freeze the clock of the DS1307
	@param run zero for stop, all other for run
*/
void ds1307_rtc(uint8_t run){
	
	uint8_t readout;
	
	// Read current value
	readout = ds1307_read(0x00);
	
	
	// Set CH bit
	if (run)
		readout &= ~(0x80);
	else
		readout |= 0x80;
		
	// Write value back
	ds1307_write(0x00, readout);
}
/******************************************************************************/

/**
	@brief Write the current time to the DS1307
	@return zero for no error, one for communication error
*/
uint8_t ds1307_setTime(void){
	uint8_t chbit = ds1307_read(0x00) & 0x80;

	// Open device for write
	if (i2c_master_open_write(DS1307_I2C_ADR))
		return 1;

	i2c_master_write(0x00);
	if (chbit)
		i2c_master_write(ds1307_encodeBcd(second) | 0x80);
	else
		i2c_master_write(ds1307_encodeBcd(second) & 0x7F);		
	
	i2c_master_write(ds1307_encodeBcd(minute));
	i2c_master_write(ds1307_encodeBcd(hour));
	
	i2c_master_write(weekday);		
	
	i2c_master_write(ds1307_encodeBcd(day));
	i2c_master_write(ds1307_encodeBcd(month));
	i2c_master_write(ds1307_encodeBcd(year));		
	
	
	// Close I2C bus
	i2c_master_close();
	
	return 0;
}
/******************************************************************************/

/**
	@brief Get the current time from the DS1307
	@return zero for no error, one for communication error
*/
uint8_t ds1307_getTime(void){

	// Open device for write
	if (i2c_master_open_write(DS1307_I2C_ADR))
		return 1;
	
	// select reading position (0x00)
	i2c_master_write(0x00);
	
	// (Re-)Open device for read
	i2c_master_open_read(DS1307_I2C_ADR);
	
	// Read value
	second = ds1307_decodeBcd(i2c_master_read_next() & 0x7F);          // decode byte to BCD for variable second
	minute = ds1307_decodeBcd(i2c_master_read_next() & 0x7F);           // decode byte to BCD for variable minute
	hour = ds1307_decodeBcd(i2c_master_read_next() & 0x7F);             // decode byte to BCD for variable hour
	weekday = ds1307_decodeBcd(i2c_master_read_next() & 0x7F);          // decode byte to BCD for variable weekday
	day = ds1307_decodeBcd(i2c_master_read_next() & 0x7F);              // decode byte to BCD for variable day
    month = ds1307_decodeBcd(i2c_master_read_next() & 0x7F);            // decode byte to BCD for variable month
	year = ds1307_decodeBcd(i2c_master_read_last() & 0x7F);             // decode byte to BCD for variable year and stop reading from memory
	// TODO minute, hour, ...
	
	// Close I2C bus
	i2c_master_close();
	
	return 0;
}
/******************************************************************************/


void nexttime(void){
	uint8_t days[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

	// Set second to zero (jump is in minutes)
	second = 0;
	
	// Goto next minute
	minute++;
	if (minute >= 60){
		minute = 0;
		hour++;
	} else {
		return;
	}
	
	// Check hour
	if (hour >= 24){
		hour = 0;
		day++;
	} else {
		return;
	}
	
	// Check for gap year
	if ((((year % 4) == 0) && ((year % 100) != 0)) || ((year % 400) == 0)){         // check if the year is a leap year
        days[2] = 29;                                                               // set the month to day2 and count 29
	} else {                                                                        
        return;                                                                     
	}
	// TODO
	
	// Check day
	if (day > days[month]){
		day= 1;
		month++;
	}
	
	// Check month
	if (month >= 12){
		month = 1;
		year++;
	}
}
/******************************************************************************/


/**
	@brief Load 8 bit value from the EEPROM
	@return loaded value
*/

uint8_t load_value8bit(uint8_t pos){
	uint8_t value;
	i2c_master_open_write(0xA0); 					
	i2c_master_write(pos);		    				
	i2c_master_open_read(0xA0);						
	value = i2c_master_read_last();				    
	i2c_master_close();								

	/* TODO */
	
	return value;
}
/******************************************************************************/


/**
	@brief Load a 16 bit value from the EEPROM
	@return loaded value
*/

uint16_t load_value16bit(uint8_t pos){
	uint8_t highbyte, lowbyte;
	i2c_master_open_write(0xA0); 					
	i2c_master_write(pos);						    
	i2c_master_open_read(0xA0);						
	lowbyte = i2c_master_read_next();				
	highbyte = i2c_master_read_last();				
	i2c_master_close();					

	/* TODO */
	
	return highbyte * 256 + lowbyte;
}
/******************************************************************************/

/**
	@brief Save a 8 bit value to the EEPROM
	@param tosave value to save
*/

void save_value8bit(uint8_t tosave, uint8_t pos){
	i2c_master_open_write(0xA0);					
	i2c_master_write(pos);						    
	i2c_master_write(tosave);

	/* TODO */

	i2c_master_close();
	_delay_ms(10); // wait 10ms to make sure that data is written
}
/******************************************************************************/


/**
	@brief Save a 16 bit value to the EEPROM
	@param tosave value to save
*/
void save_value16bit(uint16_t tosave, uint8_t pos){
	uint8_t highbyte, lowbyte;
	highbyte = tosave / 256;						// convert the value to save to 8-bit
	lowbyte = tosave % 256;							// convert the value to save to 8-bit
	i2c_master_open_write(0xA0);					// open to write 
	i2c_master_write(pos);						    // set pointer position
	i2c_master_write(lowbyte);						// write lowbyte
	i2c_master_write(highbyte);						// write highbyte
	

	i2c_master_close();
	_delay_ms(10); // wait 10ms to make sure that data is written	
}
/******************************************************************************/


/**
	@brief Read the temperature with the internal analog sensor
	@return temperature in 1/10 deg. Celsius
*/
uint16_t adc_temperature_oversample(void){
	uint8_t i;
	uint32_t sum = 0;
	
	for (i = 0; i < 128; i++){
		ADCSRA |= (1 << ADSC)| (1 << ADEN); // Start ADC
	
		while( ADCSRA & (1 << ADSC) ) // wait for ADC complete
			;
	
		sum += ADCW;
	}
	

	sum /= 32;

	// substract offset
	sum -= TEMPSENOR_OFFSET;
	
	// 0.27 deg. Celsius per step
	sum *= 27;
	sum /= 10;
	
	return sum;
}
/******************************************************************************/


void log_data(void){
	    uint8_t flag = 0;
    char str[17];
	 
    while ((second != 0) && (flag == 0)){                               
        _delay_ms(100);                                                 
        ds1307_getTime();                                               
        display_standby(adc_temperature_oversample());                  
        if ((~PINB & (1 << PB0)) && (~PINB & (1 << PB1))){              
            lcd_clear();                                                
            lcd_string("Exiting Log Mode");                             

            snprintf(str, 17, "In memory: %d", load_value8bit(0));      
            lcd_setcursor(0, 2);                                     
            lcd_string(str);                                         
            _delay_ms(1500);                                           
            return;                                                     
        }
    
        save_value8bit(count, 0);                                      
        save_value8bit(second,0x00); 
		save_value8bit(minute,0x01);
		save_value8bit((hour & 127) | (day & 1)<<7,0x02); 
		save_value8bit(((day >>1) & 15)  | (weekday << 4),0x03) ; 
		save_value8bit((month & 15)|((year & 15)<<4),0x04); 
        save_value16bit(adc_temperature_oversample(), 6);               
        save = 8;                                                      
        count++;                                                        
        lcd_clear();                                               
        lcd_string("Recording Data");                                   
        snprintf(str, 17, "No.: %d. %d.%d C", load_value8bit(0), adc_temperature_oversample()/10, adc_temperature_oversample()%10);     
        lcd_setcursor(0,2);                                            
        lcd_string(str);                                                

        _delay_ms(1500);                                               
        flag = 1;                                                       
    while(flag){
        _delay_ms(100);                                                
		ds1307_getTime();                                            
		display_standby(adc_temperature_oversample());                  
            if (second == 0){                                           
                save_value8bit(count, 0);                               
                save_value16bit(adc_temperature_oversample(), save);    
                save += 2;                                            
                count++;                                                

                lcd_clear();                                         
                lcd_string("Recording Data");                          

                snprintf(str, 16, "No.: %d. %d.%d C", load_value8bit(0), adc_temperature_oversample()/10, adc_temperature_oversample()%10);     
                lcd_setcursor(0,2);                                     
                lcd_string(str);                                        

                _delay_ms(1500);                                        
            }
        if (((~PINB & (1 << PB0)) && (~PINB & (1 << PB1))) || (load_value8bit(0) == 125)){      
            lcd_clear();                                                
            lcd_string("Exiting Log Mode");                             

            snprintf(str, 17, "In memory: %2d", load_value8bit(0));     
            lcd_setcursor(0, 2);                                        
            lcd_string(str);                                            
            _delay_ms(1500);                                            
            return;                                                     
        }
	}
}
	// TODO
}
/******************************************************************************/


void show_data(void){
	 uint8_t flag = 1, i, counter, flag2 = 0, flag3 = 0;
    uint16_t value;
    char str[17];
	
    year = ((load_value8bit(0x04)) >> 4) & 15;                                           // read start year from byte 1 and give to global variable year
    month = (load_value8bit(0x04)) & 15;                                          // read start month from byte 2 and give to global variable month
    day = ((load_value8bit(0x02)) >> 7) + (((load_value8bit(0x03)) & 15) << 1);                                            // read start day from byte 1 and give to global variable day
    hour = (load_value8bit(0x02)) & 127;                                           // read start hour from byte 1 and give to global variable hour
    minute = load_value8bit(0x01);                                         // read start minute from byte 1 and give to global variable minute
    temperature = load_value16bit(0x06);                                         // read temperature from bytes 6 and 7 and give it to variable value
    counter = 1;                                                        // set read value counter to 1
    load = 8;                                                           // set read address to 8
    while(flag){
        _delay_ms(100);                                                 
        snprintf(str, 16, "%02d/%02d/%02d %02d:%02d", year, month, day, hour, minute);      
        lcd_clear();                                                    
        lcd_string(str);                                                

        snprintf(str, 16, "No: %d.  %d.%d C", counter, value / 10, value % 10);             
        lcd_setcursor(0,2);                                             
        lcd_string(str);                                                
        if (~PINB & (1 << PB0)){                                        
            if (counter == load_value8bit(0)){                          
                flag3 = 1;                                              
            } else{                                                     
                nexttime();                                             
                value = load_value16bit(load);                          
                load += 2;                                              
                counter++;                                              
            }
            while (~PINB & (1 << PB0)){                                
                if ((~PINB & (1 << PB0)) && (~PINB & (1 << PB1))){      
                    flag2 = 1;                                          
                }
                else{                                            
                    ;                                                  
                }
            
            while (flag2){                                              
                _delay_ms(100);                                      
                lcd_clear();                                            
                lcd_string(" Reset Memory ?");                          

                lcd_setcursor(0, 2);                                   
                lcd_string("Key1: Y. Key2: N");                         

                if (~PINB & (1 << PB0)){                              
                    lcd_clear();                                        
                    for (i = 0; i < 255; i++){                          
                        _delay_ms(2);                                   
                        save_value8bit(0, i);                         
                        lcd_clear();                                  
                        lcd_string("Clearing Memory.");               
                    }
                    lcd_clear();                                        
                    lcd_string(" Memory Cleared");                     
                    lcd_setcursor(0, 2);                                
                    lcd_string("Exitng Read Mode");                     
                    _delay_ms(1500);                                    
                    return;                                            
                }
                if (~PINB & (1 << PB1)){                                
                    lcd_clear();                                      
                    lcd_string("  Read Mode Off");                    
                    _delay_ms(1500);                                    
                    return;                                         
                }
            }
            while (flag3){
                lcd_clear();                                          
                lcd_string("No More Data");                             
                _delay_ms(1500);                                       
                break;                                                  
            }
        }
    }
}
	// TODO
	return;
}
/******************************************************************************/


/**
	@brief Main function
	@return only a dummy to avoid a compiler warning, not used
*/
int main(void){
	uint16_t nowtemp;

	init(); 	// Function to initialise I/Os
	lcd_init(); // Function to initialise LCD display
	i2c_master_init(1, 10); // Init TWI
	ds1307_rtc(1); 


	// Analog Input
	ADMUX = 0;//TODO 
	ADMUX |= (1 << REFS1) | (1 << REFS0);// 1.1V as reference
	ADMUX |= (1 << MUX3);
	ADCSRA = (1 << ADPS2)| (1 << ADPS1); // ADC Prescale by 64
	ADCSRA |= (1 << ADSC)| (1 << ADEN); // Start first conversion (dummy read)


	// Loop forever
	for(;;){
		
		// Short delay
		_delay_ms(100);
		
		
		// Mesure temperature
		nowtemp = adc_temperature_oversample();
		
		
		// Load current time/date from DS1307
		ds1307_getTime(); // TODO
		
		// Show current time/date
		display_standby(nowtemp);

		// Show recorded data
		if ( ~PINB & (1 << PB0) )
			show_data();
		
		// Start Recording
		if (~PINB & (1 << PB1))
			log_data();

	}

	return 0;
}
/******************************************************************************/
