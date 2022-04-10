#define FCY 4000000UL

#include "mcc_generated_files/system.h"
#include "mcc_generated_files/adc1.h"
#include "mcc_generated_files/tmr1.h"
#include "mcc_generated_files/i2c1.h"

#include <libpic30.h>

extern void I2C1_Initialize(void);
extern void I2CStart(void);
extern void I2CRestart(void);
extern void I2CStop(void);
extern void I2CWaitACK(void);
extern void I2CIdle(void);
extern void I2CWrite(unsigned int c);
extern void I2CSequentialWriteReg(int addr, int* value, int length);

#define _RGBAddr 0x10
#define FUNC 0x02
#define COLOR 0x03
#define PIX_X 0x04
#define PIX_Y 0x05
#define BITMAP 0x06
#define STR 0x07

#define CLEAR 0x1

#define RED 1
#define GREEN 2
#define YELLOW 3
#define BLUE 4
#define WHITE 7
#define SIZE 50

int addr = 0x10; //correct address, verified by Arduino "address find" sketch
int reg; // internal register value
int length = 50; //length of message being sent to display
int buf[SIZE] = {0}; //

void I2CStop(void) {
    I2C1CONLbits.RCEN = 0; // Receive mode not in progress
    I2C1CONLbits.PEN = 1; // Stop condition
    while(I2C1CONLbits.PEN); // Wait until stop condition is idle again
}

void I2CStart(void) {
    I2C1STATbits.ACKSTAT = 1; // Reset any ACK
    I2C1CONLbits.SEN = 1; // Initiate start condition
    while(I2C1CONLbits.SEN); // wWait until start condition is idle again
}

void I2CWaitACK(void) {
    while(I2C1STATbits.ACKSTAT); //Wait until acknowledge received from client
}

void I2CIdle(void) {
    while(I2C1STATbits.TRSTAT); //wait until host transmit is no longer in progress
}

void I2CWrite(unsigned int value) {
    I2C1TRN = value; //automatically cleared?
    I2CWaitACK(); //Wait for and verify an Acknowledge from the client
}

void I2CSequentialWriteReg(int reg, int* value, int length) {
    int j;
    // Initiate start condition
    I2CStart();
    // Write the address of the display
    I2CWrite((addr<<1)&0xFE);
    // Wait until previous write is finished
    I2CIdle();
    // Write to the register targeted in display
    I2CWrite(reg);
    // Wait until previous write is finished
    I2CIdle();
    // Send actual data to display
    for(j = 0; j < length; j++) {
        // I2CWrite(value[j]);
        I2CWrite(*value); // Write value in LSB
        value++; // Move to next bit
        I2CIdle(); // Move on once write is complete
    }
    // Initiate stop condition
    I2CStop();
}

void displayColor(int x, int y, int color) {
    // Turn on a single pixel
    buf[0] = (buf[0] & (0xe6)) | (0x01 << 3);
    buf[1] = color;
    buf[2] = x; // X POSITION
    buf[3] = y; // Y POSITION
    I2CSequentialWriteReg(0x02,buf,SIZE);
}

int applyADC(int delay) {
    ADC1_Enable();
    ADC1_ChannelSelect(emgPin);
    ADC1_SoftwareTriggerEnable();
    __delay_ms(delay);
    ADC1_SoftwareTriggerDisable();
    while(!ADC1_IsConversionComplete(emgPin));
    int converted_value = ADC1_ConversionResultGet(emgPin);
    ADC1_Disable(); 
    
    return converted_value;
}

int counter = 0;
int pulseWidth = 2; // 2 = 1ms duty cycle, 3 = 2ms duty cycle
bool specialFlag = false;
int lowCounter = 0;
int lowPulseWidth = 19;

void Timer1InterruptHandler(void);
void Timer1InterruptHandler(void) {
    if(specialFlag) {
        lowCounter++;
        if(lowCounter < lowPulseWidth) {
            return;
        } else {
            specialFlag = false;
            lowCounter = 0;
            counter = 0;
        }
    }
    counter++; 
    LATCbits.LATC12 = 1;
    if(counter >= pulseWidth) {
        LATCbits.LATC12 = 0;
        specialFlag = true;
    }
}

void clearDisplay(void) {
    int i = 0;
    buf[0] = 0x1;
    
    for(i = 1; i < SIZE; i++) {
        buf[i] = 0;
    }
    
    I2CSequentialWriteReg(0x02, buf, SIZE);
}

void setPins(void) {
    TRISCbits.TRISC12 = 0; // PWM_2 (Servo Motor)
    
    TRISBbits.TRISB12 = 0; // TX_1 (Red LED)
    TRISCbits.TRISC15 = 0; // RX_1 (Yellow LED)
    TRISCbits.TRISC14 = 0; // INT_1 (Green LED)
    TRISCbits.TRISC13 = 0; // PWM_1 (Blue LED)
}

void setDisplayWhite(void) {
    buf[0] = 0x1;
    buf[0] = (buf[0] & (0xe7)) | (0x01 << 3);
    buf[1] = WHITE;
    buf[2] = 0;
    buf[3] = 0;
    I2CSequentialWriteReg(0x02,buf,SIZE);
}

void useDebuggingLEDs(int digital_emg_signal_value) {
    if(digital_emg_signal_value >= 0 && digital_emg_signal_value <= 1023) {
        LATBbits.LATB12 = 1;
        LATCbits.LATC15 = 0;
        LATCbits.LATC14 = 0;
        LATCbits.LATC13 = 0;
    } else if(digital_emg_signal_value >= 1024 && digital_emg_signal_value <= 2047) {
        LATBbits.LATB12 = 0;
        LATCbits.LATC15 = 1;
        LATCbits.LATC14 = 0;
        LATCbits.LATC13 = 0;
    } else if(digital_emg_signal_value >= 2048 && digital_emg_signal_value <= 3071) {
        LATBbits.LATB12 = 0;
        LATCbits.LATC15 = 0;
        LATCbits.LATC14 = 1;
        LATCbits.LATC13 = 0;
    } else if(digital_emg_signal_value >= 3072 && digital_emg_signal_value <= 4095) {
        LATBbits.LATB12 = 0;
        LATCbits.LATC15 = 0;
        LATCbits.LATC14 = 0;
        LATCbits.LATC13 = 1;
    }
}

void moveServoMotor(int digital_emg_signal_value) {
    if(digital_emg_signal_value >= 3500) {
        pulseWidth = 2;
        lowPulseWidth = 19;
    } else if(digital_emg_signal_value <= 2000) {
        pulseWidth = 3;
        lowPulseWidth = 18;
    }
}

int main(void) {
    SYSTEM_Initialize();
    TMR1_SetInterruptHandler(Timer1InterruptHandler);
    setPins();
    
    int digital_emg_signal_value = 0;
    while(1) {
        clearDisplay();
        setDisplayWhite();
        
        digital_emg_signal_value = applyADC(1);
        
        useDebuggingLEDs(digital_emg_signal_value);
        moveServoMotor(digital_emg_signal_value);
        
        if(digital_emg_signal_value >= 0 && digital_emg_signal_value <= 255) {
            displayColor(0,7,RED);
            displayColor(1,7,WHITE);
            displayColor(2,7,WHITE);
            displayColor(2,6,WHITE);
            displayColor(3,7,WHITE);
            displayColor(3,6,WHITE);
            displayColor(4,7,WHITE);
            displayColor(4,6,WHITE);
            displayColor(4,5,WHITE);
            displayColor(5,7,WHITE);
            displayColor(5,6,WHITE);
            displayColor(5,5,WHITE);
            displayColor(6,7,WHITE);
            displayColor(6,6,WHITE);
            displayColor(6,5,WHITE);
            displayColor(6,4,WHITE);
            displayColor(7,7,WHITE);
            displayColor(7,6,WHITE);
            displayColor(7,5,WHITE);
            displayColor(7,4,WHITE);
            displayColor(8,7,WHITE);
            displayColor(8,6,WHITE);
            displayColor(8,5,WHITE);
            displayColor(8,4,WHITE);
            displayColor(8,3,WHITE);
            displayColor(9,7,WHITE);
            displayColor(9,6,WHITE);
            displayColor(9,5,WHITE);
            displayColor(9,4,WHITE);
            displayColor(9,3,WHITE);
            displayColor(10,7,WHITE);
            displayColor(10,6,WHITE);
            displayColor(10,5,WHITE);
            displayColor(10,4,WHITE);
            displayColor(10,3,WHITE);
            displayColor(10,2,WHITE);
            displayColor(11,7,WHITE);
            displayColor(11,6,WHITE);
            displayColor(11,5,WHITE);
            displayColor(11,4,WHITE);
            displayColor(11,3,WHITE);
            displayColor(11,2,WHITE);
            displayColor(12,7,WHITE);
            displayColor(12,6,WHITE);
            displayColor(12,5,WHITE);
            displayColor(12,4,WHITE);
            displayColor(12,3,WHITE);
            displayColor(12,2,WHITE);
            displayColor(12,1,WHITE);
            displayColor(13,7,WHITE);
            displayColor(13,6,WHITE);
            displayColor(13,5,WHITE);
            displayColor(13,4,WHITE);
            displayColor(13,3,WHITE);
            displayColor(13,2,WHITE);
            displayColor(13,1,WHITE);
            displayColor(14,7,WHITE);
            displayColor(14,6,WHITE);
            displayColor(14,5,WHITE);
            displayColor(14,4,WHITE);
            displayColor(14,3,WHITE);
            displayColor(14,2,WHITE);
            displayColor(14,1,WHITE);
            displayColor(14,0,WHITE);
            displayColor(15,7,WHITE);
            displayColor(15,6,WHITE);
            displayColor(15,5,WHITE);
            displayColor(15,4,WHITE);
            displayColor(15,3,WHITE);
            displayColor(15,2,WHITE);
            displayColor(15,1,WHITE);
            displayColor(15,0,WHITE);
        } else if(digital_emg_signal_value >= 256 && digital_emg_signal_value <= 511) {
            displayColor(0,7,RED);
            displayColor(1,7,RED);
            displayColor(2,7,WHITE);
            displayColor(2,6,WHITE);
            displayColor(3,7,WHITE);
            displayColor(3,6,WHITE);
            displayColor(4,7,WHITE);
            displayColor(4,6,WHITE);
            displayColor(4,5,WHITE);
            displayColor(5,7,WHITE);
            displayColor(5,6,WHITE);
            displayColor(5,5,WHITE);
            displayColor(6,7,WHITE);
            displayColor(6,6,WHITE);
            displayColor(6,5,WHITE);
            displayColor(6,4,WHITE);
            displayColor(7,7,WHITE);
            displayColor(7,6,WHITE);
            displayColor(7,5,WHITE);
            displayColor(7,4,WHITE);
            displayColor(8,7,WHITE);
            displayColor(8,6,WHITE);
            displayColor(8,5,WHITE);
            displayColor(8,4,WHITE);
            displayColor(8,3,WHITE);
            displayColor(9,7,WHITE);
            displayColor(9,6,WHITE);
            displayColor(9,5,WHITE);
            displayColor(9,4,WHITE);
            displayColor(9,3,WHITE);
            displayColor(10,7,WHITE);
            displayColor(10,6,WHITE);
            displayColor(10,5,WHITE);
            displayColor(10,4,WHITE);
            displayColor(10,3,WHITE);
            displayColor(10,2,WHITE);
            displayColor(11,7,WHITE);
            displayColor(11,6,WHITE);
            displayColor(11,5,WHITE);
            displayColor(11,4,WHITE);
            displayColor(11,3,WHITE);
            displayColor(11,2,WHITE);
            displayColor(12,7,WHITE);
            displayColor(12,6,WHITE);
            displayColor(12,5,WHITE);
            displayColor(12,4,WHITE);
            displayColor(12,3,WHITE);
            displayColor(12,2,WHITE);
            displayColor(12,1,WHITE);
            displayColor(13,7,WHITE);
            displayColor(13,6,WHITE);
            displayColor(13,5,WHITE);
            displayColor(13,4,WHITE);
            displayColor(13,3,WHITE);
            displayColor(13,2,WHITE);
            displayColor(13,1,WHITE);
            displayColor(14,7,WHITE);
            displayColor(14,6,WHITE);
            displayColor(14,5,WHITE);
            displayColor(14,4,WHITE);
            displayColor(14,3,WHITE);
            displayColor(14,2,WHITE);
            displayColor(14,1,WHITE);
            displayColor(14,0,WHITE);
            displayColor(15,7,WHITE);
            displayColor(15,6,WHITE);
            displayColor(15,5,WHITE);
            displayColor(15,4,WHITE);
            displayColor(15,3,WHITE);
            displayColor(15,2,WHITE);
            displayColor(15,1,WHITE);
            displayColor(15,0,WHITE);
        } else if(digital_emg_signal_value >= 512 && digital_emg_signal_value <= 767) {
            displayColor(0,7,RED);
            displayColor(1,7,RED);
            displayColor(2,7,RED);
            displayColor(2,6,RED);
            displayColor(3,7,WHITE);
            displayColor(3,6,WHITE);
            displayColor(4,7,WHITE);
            displayColor(4,6,WHITE);
            displayColor(4,5,WHITE);
            displayColor(5,7,WHITE);
            displayColor(5,6,WHITE);
            displayColor(5,5,WHITE);
            displayColor(6,7,WHITE);
            displayColor(6,6,WHITE);
            displayColor(6,5,WHITE);
            displayColor(6,4,WHITE);
            displayColor(7,7,WHITE);
            displayColor(7,6,WHITE);
            displayColor(7,5,WHITE);
            displayColor(7,4,WHITE);
            displayColor(8,7,WHITE);
            displayColor(8,6,WHITE);
            displayColor(8,5,WHITE);
            displayColor(8,4,WHITE);
            displayColor(8,3,WHITE);
            displayColor(9,7,WHITE);
            displayColor(9,6,WHITE);
            displayColor(9,5,WHITE);
            displayColor(9,4,WHITE);
            displayColor(9,3,WHITE);
            displayColor(10,7,WHITE);
            displayColor(10,6,WHITE);
            displayColor(10,5,WHITE);
            displayColor(10,4,WHITE);
            displayColor(10,3,WHITE);
            displayColor(10,2,WHITE);
            displayColor(11,7,WHITE);
            displayColor(11,6,WHITE);
            displayColor(11,5,WHITE);
            displayColor(11,4,WHITE);
            displayColor(11,3,WHITE);
            displayColor(11,2,WHITE);
            displayColor(12,7,WHITE);
            displayColor(12,6,WHITE);
            displayColor(12,5,WHITE);
            displayColor(12,4,WHITE);
            displayColor(12,3,WHITE);
            displayColor(12,2,WHITE);
            displayColor(12,1,WHITE);
            displayColor(13,7,WHITE);
            displayColor(13,6,WHITE);
            displayColor(13,5,WHITE);
            displayColor(13,4,WHITE);
            displayColor(13,3,WHITE);
            displayColor(13,2,WHITE);
            displayColor(13,1,WHITE);
            displayColor(14,7,WHITE);
            displayColor(14,6,WHITE);
            displayColor(14,5,WHITE);
            displayColor(14,4,WHITE);
            displayColor(14,3,WHITE);
            displayColor(14,2,WHITE);
            displayColor(14,1,WHITE);
            displayColor(14,0,WHITE);
            displayColor(15,7,WHITE);
            displayColor(15,6,WHITE);
            displayColor(15,5,WHITE);
            displayColor(15,4,WHITE);
            displayColor(15,3,WHITE);
            displayColor(15,2,WHITE);
            displayColor(15,1,WHITE);
            displayColor(15,0,WHITE);
        } else if(digital_emg_signal_value >= 768 && digital_emg_signal_value <= 1023) {
            displayColor(0,7,RED);
            displayColor(1,7,RED);
            displayColor(2,7,RED);
            displayColor(2,6,RED);
            displayColor(3,7,RED);
            displayColor(3,6,RED);
            displayColor(4,7,WHITE);
            displayColor(4,6,WHITE);
            displayColor(4,5,WHITE);
            displayColor(5,7,WHITE);
            displayColor(5,6,WHITE);
            displayColor(5,5,WHITE);
            displayColor(6,7,WHITE);
            displayColor(6,6,WHITE);
            displayColor(6,5,WHITE);
            displayColor(6,4,WHITE);
            displayColor(7,7,WHITE);
            displayColor(7,6,WHITE);
            displayColor(7,5,WHITE);
            displayColor(7,4,WHITE);
            displayColor(8,7,WHITE);
            displayColor(8,6,WHITE);
            displayColor(8,5,WHITE);
            displayColor(8,4,WHITE);
            displayColor(8,3,WHITE);
            displayColor(9,7,WHITE);
            displayColor(9,6,WHITE);
            displayColor(9,5,WHITE);
            displayColor(9,4,WHITE);
            displayColor(9,3,WHITE);
            displayColor(10,7,WHITE);
            displayColor(10,6,WHITE);
            displayColor(10,5,WHITE);
            displayColor(10,4,WHITE);
            displayColor(10,3,WHITE);
            displayColor(10,2,WHITE);
            displayColor(11,7,WHITE);
            displayColor(11,6,WHITE);
            displayColor(11,5,WHITE);
            displayColor(11,4,WHITE);
            displayColor(11,3,WHITE);
            displayColor(11,2,WHITE);
            displayColor(12,7,WHITE);
            displayColor(12,6,WHITE);
            displayColor(12,5,WHITE);
            displayColor(12,4,WHITE);
            displayColor(12,3,WHITE);
            displayColor(12,2,WHITE);
            displayColor(12,1,WHITE);
            displayColor(13,7,WHITE);
            displayColor(13,6,WHITE);
            displayColor(13,5,WHITE);
            displayColor(13,4,WHITE);
            displayColor(13,3,WHITE);
            displayColor(13,2,WHITE);
            displayColor(13,1,WHITE);
            displayColor(14,7,WHITE);
            displayColor(14,6,WHITE);
            displayColor(14,5,WHITE);
            displayColor(14,4,WHITE);
            displayColor(14,3,WHITE);
            displayColor(14,2,WHITE);
            displayColor(14,1,WHITE);
            displayColor(14,0,WHITE);
            displayColor(15,7,WHITE);
            displayColor(15,6,WHITE);
            displayColor(15,5,WHITE);
            displayColor(15,4,WHITE);
            displayColor(15,3,WHITE);
            displayColor(15,2,WHITE);
            displayColor(15,1,WHITE);
            displayColor(15,0,WHITE);
        } else if(digital_emg_signal_value >= 1024 && digital_emg_signal_value <= 1279) {
            displayColor(0,7,RED);
            displayColor(1,7,RED);
            displayColor(2,7,RED);
            displayColor(2,6,RED);
            displayColor(3,7,RED);
            displayColor(3,6,RED);
            displayColor(4,7,YELLOW);
            displayColor(4,6,YELLOW);
            displayColor(4,5,YELLOW);
            displayColor(5,7,WHITE);
            displayColor(5,6,WHITE);
            displayColor(5,5,WHITE);
            displayColor(6,7,WHITE);
            displayColor(6,6,WHITE);
            displayColor(6,5,WHITE);
            displayColor(6,4,WHITE);
            displayColor(7,7,WHITE);
            displayColor(7,6,WHITE);
            displayColor(7,5,WHITE);
            displayColor(7,4,WHITE);
            displayColor(8,7,WHITE);
            displayColor(8,6,WHITE);
            displayColor(8,5,WHITE);
            displayColor(8,4,WHITE);
            displayColor(8,3,WHITE);
            displayColor(9,7,WHITE);
            displayColor(9,6,WHITE);
            displayColor(9,5,WHITE);
            displayColor(9,4,WHITE);
            displayColor(9,3,WHITE);
            displayColor(10,7,WHITE);
            displayColor(10,6,WHITE);
            displayColor(10,5,WHITE);
            displayColor(10,4,WHITE);
            displayColor(10,3,WHITE);
            displayColor(10,2,WHITE);
            displayColor(11,7,WHITE);
            displayColor(11,6,WHITE);
            displayColor(11,5,WHITE);
            displayColor(11,4,WHITE);
            displayColor(11,3,WHITE);
            displayColor(11,2,WHITE);
            displayColor(12,7,WHITE);
            displayColor(12,6,WHITE);
            displayColor(12,5,WHITE);
            displayColor(12,4,WHITE);
            displayColor(12,3,WHITE);
            displayColor(12,2,WHITE);
            displayColor(12,1,WHITE);
            displayColor(13,7,WHITE);
            displayColor(13,6,WHITE);
            displayColor(13,5,WHITE);
            displayColor(13,4,WHITE);
            displayColor(13,3,WHITE);
            displayColor(13,2,WHITE);
            displayColor(13,1,WHITE);
            displayColor(14,7,WHITE);
            displayColor(14,6,WHITE);
            displayColor(14,5,WHITE);
            displayColor(14,4,WHITE);
            displayColor(14,3,WHITE);
            displayColor(14,2,WHITE);
            displayColor(14,1,WHITE);
            displayColor(14,0,WHITE);
            displayColor(15,7,WHITE);
            displayColor(15,6,WHITE);
            displayColor(15,5,WHITE);
            displayColor(15,4,WHITE);
            displayColor(15,3,WHITE);
            displayColor(15,2,WHITE);
            displayColor(15,1,WHITE);
            displayColor(15,0,WHITE);
        } else if(digital_emg_signal_value >= 1280 && digital_emg_signal_value <= 1535) {
            displayColor(0,7,RED);
            displayColor(1,7,RED);
            displayColor(2,7,RED);
            displayColor(2,6,RED);
            displayColor(3,7,RED);
            displayColor(3,6,RED);
            displayColor(4,7,YELLOW);
            displayColor(4,6,YELLOW);
            displayColor(4,5,YELLOW);
            displayColor(5,7,YELLOW);
            displayColor(5,6,YELLOW);
            displayColor(5,5,YELLOW);
            displayColor(6,7,WHITE);
            displayColor(6,6,WHITE);
            displayColor(6,5,WHITE);
            displayColor(6,4,WHITE);
            displayColor(7,7,WHITE);
            displayColor(7,6,WHITE);
            displayColor(7,5,WHITE);
            displayColor(7,4,WHITE);
            displayColor(8,7,WHITE);
            displayColor(8,6,WHITE);
            displayColor(8,5,WHITE);
            displayColor(8,4,WHITE);
            displayColor(8,3,WHITE);
            displayColor(9,7,WHITE);
            displayColor(9,6,WHITE);
            displayColor(9,5,WHITE);
            displayColor(9,4,WHITE);
            displayColor(9,3,WHITE);
            displayColor(10,7,WHITE);
            displayColor(10,6,WHITE);
            displayColor(10,5,WHITE);
            displayColor(10,4,WHITE);
            displayColor(10,3,WHITE);
            displayColor(10,2,WHITE);
            displayColor(11,7,WHITE);
            displayColor(11,6,WHITE);
            displayColor(11,5,WHITE);
            displayColor(11,4,WHITE);
            displayColor(11,3,WHITE);
            displayColor(11,2,WHITE);
            displayColor(12,7,WHITE);
            displayColor(12,6,WHITE);
            displayColor(12,5,WHITE);
            displayColor(12,4,WHITE);
            displayColor(12,3,WHITE);
            displayColor(12,2,WHITE);
            displayColor(12,1,WHITE);
            displayColor(13,7,WHITE);
            displayColor(13,6,WHITE);
            displayColor(13,5,WHITE);
            displayColor(13,4,WHITE);
            displayColor(13,3,WHITE);
            displayColor(13,2,WHITE);
            displayColor(13,1,WHITE);
            displayColor(14,7,WHITE);
            displayColor(14,6,WHITE);
            displayColor(14,5,WHITE);
            displayColor(14,4,WHITE);
            displayColor(14,3,WHITE);
            displayColor(14,2,WHITE);
            displayColor(14,1,WHITE);
            displayColor(14,0,WHITE);
            displayColor(15,7,WHITE);
            displayColor(15,6,WHITE);
            displayColor(15,5,WHITE);
            displayColor(15,4,WHITE);
            displayColor(15,3,WHITE);
            displayColor(15,2,WHITE);
            displayColor(15,1,WHITE);
            displayColor(15,0,WHITE);
        } else if(digital_emg_signal_value >= 1536 && digital_emg_signal_value <= 1791) {
            displayColor(0,7,RED);
            displayColor(1,7,RED);
            displayColor(2,7,RED);
            displayColor(2,6,RED);
            displayColor(3,7,RED);
            displayColor(3,6,RED);
            displayColor(4,7,YELLOW);
            displayColor(4,6,YELLOW);
            displayColor(4,5,YELLOW);
            displayColor(5,7,YELLOW);
            displayColor(5,6,YELLOW);
            displayColor(5,5,YELLOW);
            displayColor(6,7,YELLOW);
            displayColor(6,6,YELLOW);
            displayColor(6,5,YELLOW);
            displayColor(6,4,YELLOW);
            displayColor(7,7,WHITE);
            displayColor(7,6,WHITE);
            displayColor(7,5,WHITE);
            displayColor(7,4,WHITE);
            displayColor(8,7,WHITE);
            displayColor(8,6,WHITE);
            displayColor(8,5,WHITE);
            displayColor(8,4,WHITE);
            displayColor(8,3,WHITE);
            displayColor(9,7,WHITE);
            displayColor(9,6,WHITE);
            displayColor(9,5,WHITE);
            displayColor(9,4,WHITE);
            displayColor(9,3,WHITE);
            displayColor(10,7,WHITE);
            displayColor(10,6,WHITE);
            displayColor(10,5,WHITE);
            displayColor(10,4,WHITE);
            displayColor(10,3,WHITE);
            displayColor(10,2,WHITE);
            displayColor(11,7,WHITE);
            displayColor(11,6,WHITE);
            displayColor(11,5,WHITE);
            displayColor(11,4,WHITE);
            displayColor(11,3,WHITE);
            displayColor(11,2,WHITE);
            displayColor(12,7,WHITE);
            displayColor(12,6,WHITE);
            displayColor(12,5,WHITE);
            displayColor(12,4,WHITE);
            displayColor(12,3,WHITE);
            displayColor(12,2,WHITE);
            displayColor(12,1,WHITE);
            displayColor(13,7,WHITE);
            displayColor(13,6,WHITE);
            displayColor(13,5,WHITE);
            displayColor(13,4,WHITE);
            displayColor(13,3,WHITE);
            displayColor(13,2,WHITE);
            displayColor(13,1,WHITE);
            displayColor(14,7,WHITE);
            displayColor(14,6,WHITE);
            displayColor(14,5,WHITE);
            displayColor(14,4,WHITE);
            displayColor(14,3,WHITE);
            displayColor(14,2,WHITE);
            displayColor(14,1,WHITE);
            displayColor(14,0,WHITE);
            displayColor(15,7,WHITE);
            displayColor(15,6,WHITE);
            displayColor(15,5,WHITE);
            displayColor(15,4,WHITE);
            displayColor(15,3,WHITE);
            displayColor(15,2,WHITE);
            displayColor(15,1,WHITE);
            displayColor(15,0,WHITE);
        } else if(digital_emg_signal_value >= 1792 && digital_emg_signal_value <= 2047) {
            displayColor(0,7,RED);
            displayColor(1,7,RED);
            displayColor(2,7,RED);
            displayColor(2,6,RED);
            displayColor(3,7,RED);
            displayColor(3,6,RED);
            displayColor(4,7,YELLOW);
            displayColor(4,6,YELLOW);
            displayColor(4,5,YELLOW);
            displayColor(5,7,YELLOW);
            displayColor(5,6,YELLOW);
            displayColor(5,5,YELLOW);
            displayColor(6,7,YELLOW);
            displayColor(6,6,YELLOW);
            displayColor(6,5,YELLOW);
            displayColor(6,4,YELLOW);
            displayColor(7,7,YELLOW);
            displayColor(7,6,YELLOW);
            displayColor(7,5,YELLOW);
            displayColor(7,4,YELLOW);
            displayColor(8,7,WHITE);
            displayColor(8,6,WHITE);
            displayColor(8,5,WHITE);
            displayColor(8,4,WHITE);
            displayColor(8,3,WHITE);
            displayColor(9,7,WHITE);
            displayColor(9,6,WHITE);
            displayColor(9,5,WHITE);
            displayColor(9,4,WHITE);
            displayColor(9,3,WHITE);
            displayColor(10,7,WHITE);
            displayColor(10,6,WHITE);
            displayColor(10,5,WHITE);
            displayColor(10,4,WHITE);
            displayColor(10,3,WHITE);
            displayColor(10,2,WHITE);
            displayColor(11,7,WHITE);
            displayColor(11,6,WHITE);
            displayColor(11,5,WHITE);
            displayColor(11,4,WHITE);
            displayColor(11,3,WHITE);
            displayColor(11,2,WHITE);
            displayColor(12,7,WHITE);
            displayColor(12,6,WHITE);
            displayColor(12,5,WHITE);
            displayColor(12,4,WHITE);
            displayColor(12,3,WHITE);
            displayColor(12,2,WHITE);
            displayColor(12,1,WHITE);
            displayColor(13,7,WHITE);
            displayColor(13,6,WHITE);
            displayColor(13,5,WHITE);
            displayColor(13,4,WHITE);
            displayColor(13,3,WHITE);
            displayColor(13,2,WHITE);
            displayColor(13,1,WHITE);
            displayColor(14,7,WHITE);
            displayColor(14,6,WHITE);
            displayColor(14,5,WHITE);
            displayColor(14,4,WHITE);
            displayColor(14,3,WHITE);
            displayColor(14,2,WHITE);
            displayColor(14,1,WHITE);
            displayColor(14,0,WHITE);
            displayColor(15,7,WHITE);
            displayColor(15,6,WHITE);
            displayColor(15,5,WHITE);
            displayColor(15,4,WHITE);
            displayColor(15,3,WHITE);
            displayColor(15,2,WHITE);
            displayColor(15,1,WHITE);
            displayColor(15,0,WHITE);
        } else if(digital_emg_signal_value >= 2048 && digital_emg_signal_value <= 2303) {
            displayColor(0,7,RED);
            displayColor(1,7,RED);
            displayColor(2,7,RED);
            displayColor(2,6,RED);
            displayColor(3,7,RED);
            displayColor(3,6,RED);
            displayColor(4,7,YELLOW);
            displayColor(4,6,YELLOW);
            displayColor(4,5,YELLOW);
            displayColor(5,7,YELLOW);
            displayColor(5,6,YELLOW);
            displayColor(5,5,YELLOW);
            displayColor(6,7,YELLOW);
            displayColor(6,6,YELLOW);
            displayColor(6,5,YELLOW);
            displayColor(6,4,YELLOW);
            displayColor(7,7,YELLOW);
            displayColor(7,6,YELLOW);
            displayColor(7,5,YELLOW);
            displayColor(7,4,YELLOW);
            displayColor(8,7,BLUE);
            displayColor(8,6,BLUE);
            displayColor(8,5,BLUE);
            displayColor(8,4,BLUE);
            displayColor(8,3,BLUE);
            displayColor(9,7,WHITE);
            displayColor(9,6,WHITE);
            displayColor(9,5,WHITE);
            displayColor(9,4,WHITE);
            displayColor(9,3,WHITE);
            displayColor(10,7,WHITE);
            displayColor(10,6,WHITE);
            displayColor(10,5,WHITE);
            displayColor(10,4,WHITE);
            displayColor(10,3,WHITE);
            displayColor(10,2,WHITE);
            displayColor(11,7,WHITE);
            displayColor(11,6,WHITE);
            displayColor(11,5,WHITE);
            displayColor(11,4,WHITE);
            displayColor(11,3,WHITE);
            displayColor(11,2,WHITE);
            displayColor(12,7,WHITE);
            displayColor(12,6,WHITE);
            displayColor(12,5,WHITE);
            displayColor(12,4,WHITE);
            displayColor(12,3,WHITE);
            displayColor(12,2,WHITE);
            displayColor(12,1,WHITE);
            displayColor(13,7,WHITE);
            displayColor(13,6,WHITE);
            displayColor(13,5,WHITE);
            displayColor(13,4,WHITE);
            displayColor(13,3,WHITE);
            displayColor(13,2,WHITE);
            displayColor(13,1,WHITE);
            displayColor(14,7,WHITE);
            displayColor(14,6,WHITE);
            displayColor(14,5,WHITE);
            displayColor(14,4,WHITE);
            displayColor(14,3,WHITE);
            displayColor(14,2,WHITE);
            displayColor(14,1,WHITE);
            displayColor(14,0,WHITE);
            displayColor(15,7,WHITE);
            displayColor(15,6,WHITE);
            displayColor(15,5,WHITE);
            displayColor(15,4,WHITE);
            displayColor(15,3,WHITE);
            displayColor(15,2,WHITE);
            displayColor(15,1,WHITE);
            displayColor(15,0,WHITE);
        } else if(digital_emg_signal_value >= 2304 && digital_emg_signal_value <= 2559) {
            displayColor(0,7,RED);
            displayColor(1,7,RED);
            displayColor(2,7,RED);
            displayColor(2,6,RED);
            displayColor(3,7,RED);
            displayColor(3,6,RED);
            displayColor(4,7,YELLOW);
            displayColor(4,6,YELLOW);
            displayColor(4,5,YELLOW);
            displayColor(5,7,YELLOW);
            displayColor(5,6,YELLOW);
            displayColor(5,5,YELLOW);
            displayColor(6,7,YELLOW);
            displayColor(6,6,YELLOW);
            displayColor(6,5,YELLOW);
            displayColor(6,4,YELLOW);
            displayColor(7,7,YELLOW);
            displayColor(7,6,YELLOW);
            displayColor(7,5,YELLOW);
            displayColor(7,4,YELLOW);
            displayColor(8,7,BLUE);
            displayColor(8,6,BLUE);
            displayColor(8,5,BLUE);
            displayColor(8,4,BLUE);
            displayColor(8,3,BLUE);
            displayColor(9,7,BLUE);
            displayColor(9,6,BLUE);
            displayColor(9,5,BLUE);
            displayColor(9,4,BLUE);
            displayColor(9,3,BLUE);
            displayColor(10,7,WHITE);
            displayColor(10,6,WHITE);
            displayColor(10,5,WHITE);
            displayColor(10,4,WHITE);
            displayColor(10,3,WHITE);
            displayColor(10,2,WHITE);
            displayColor(11,7,WHITE);
            displayColor(11,6,WHITE);
            displayColor(11,5,WHITE);
            displayColor(11,4,WHITE);
            displayColor(11,3,WHITE);
            displayColor(11,2,WHITE);
            displayColor(12,7,WHITE);
            displayColor(12,6,WHITE);
            displayColor(12,5,WHITE);
            displayColor(12,4,WHITE);
            displayColor(12,3,WHITE);
            displayColor(12,2,WHITE);
            displayColor(12,1,WHITE);
            displayColor(13,7,WHITE);
            displayColor(13,6,WHITE);
            displayColor(13,5,WHITE);
            displayColor(13,4,WHITE);
            displayColor(13,3,WHITE);
            displayColor(13,2,WHITE);
            displayColor(13,1,WHITE);
            displayColor(14,7,WHITE);
            displayColor(14,6,WHITE);
            displayColor(14,5,WHITE);
            displayColor(14,4,WHITE);
            displayColor(14,3,WHITE);
            displayColor(14,2,WHITE);
            displayColor(14,1,WHITE);
            displayColor(14,0,WHITE);
            displayColor(15,7,WHITE);
            displayColor(15,6,WHITE);
            displayColor(15,5,WHITE);
            displayColor(15,4,WHITE);
            displayColor(15,3,WHITE);
            displayColor(15,2,WHITE);
            displayColor(15,1,WHITE);
            displayColor(15,0,WHITE);
        } else if(digital_emg_signal_value >= 2560 && digital_emg_signal_value <= 2815) {
            displayColor(0,7,RED);
            displayColor(1,7,RED);
            displayColor(2,7,RED);
            displayColor(2,6,RED);
            displayColor(3,7,RED);
            displayColor(3,6,RED);
            displayColor(4,7,YELLOW);
            displayColor(4,6,YELLOW);
            displayColor(4,5,YELLOW);
            displayColor(5,7,YELLOW);
            displayColor(5,6,YELLOW);
            displayColor(5,5,YELLOW);
            displayColor(6,7,YELLOW);
            displayColor(6,6,YELLOW);
            displayColor(6,5,YELLOW);
            displayColor(6,4,YELLOW);
            displayColor(7,7,YELLOW);
            displayColor(7,6,YELLOW);
            displayColor(7,5,YELLOW);
            displayColor(7,4,YELLOW);
            displayColor(8,7,BLUE);
            displayColor(8,6,BLUE);
            displayColor(8,5,BLUE);
            displayColor(8,4,BLUE);
            displayColor(8,3,BLUE);
            displayColor(9,7,BLUE);
            displayColor(9,6,BLUE);
            displayColor(9,5,BLUE);
            displayColor(9,4,BLUE);
            displayColor(9,3,BLUE);
            displayColor(10,7,BLUE);
            displayColor(10,6,BLUE);
            displayColor(10,5,BLUE);
            displayColor(10,4,BLUE);
            displayColor(10,3,BLUE);
            displayColor(10,2,BLUE);
            displayColor(11,7,WHITE);
            displayColor(11,6,WHITE);
            displayColor(11,5,WHITE);
            displayColor(11,4,WHITE);
            displayColor(11,3,WHITE);
            displayColor(11,2,WHITE);
            displayColor(12,7,WHITE);
            displayColor(12,6,WHITE);
            displayColor(12,5,WHITE);
            displayColor(12,4,WHITE);
            displayColor(12,3,WHITE);
            displayColor(12,2,WHITE);
            displayColor(12,1,WHITE);
            displayColor(13,7,WHITE);
            displayColor(13,6,WHITE);
            displayColor(13,5,WHITE);
            displayColor(13,4,WHITE);
            displayColor(13,3,WHITE);
            displayColor(13,2,WHITE);
            displayColor(13,1,WHITE);
            displayColor(14,7,WHITE);
            displayColor(14,6,WHITE);
            displayColor(14,5,WHITE);
            displayColor(14,4,WHITE);
            displayColor(14,3,WHITE);
            displayColor(14,2,WHITE);
            displayColor(14,1,WHITE);
            displayColor(14,0,WHITE);
            displayColor(15,7,WHITE);
            displayColor(15,6,WHITE);
            displayColor(15,5,WHITE);
            displayColor(15,4,WHITE);
            displayColor(15,3,WHITE);
            displayColor(15,2,WHITE);
            displayColor(15,1,WHITE);
            displayColor(15,0,WHITE);
        } else if(digital_emg_signal_value >= 2816 && digital_emg_signal_value <= 3071) {
            displayColor(0,7,RED);
            displayColor(1,7,RED);
            displayColor(2,7,RED);
            displayColor(2,6,RED);
            displayColor(3,7,RED);
            displayColor(3,6,RED);
            displayColor(4,7,YELLOW);
            displayColor(4,6,YELLOW);
            displayColor(4,5,YELLOW);
            displayColor(5,7,YELLOW);
            displayColor(5,6,YELLOW);
            displayColor(5,5,YELLOW);
            displayColor(6,7,YELLOW);
            displayColor(6,6,YELLOW);
            displayColor(6,5,YELLOW);
            displayColor(6,4,YELLOW);
            displayColor(7,7,YELLOW);
            displayColor(7,6,YELLOW);
            displayColor(7,5,YELLOW);
            displayColor(7,4,YELLOW);
            displayColor(8,7,BLUE);
            displayColor(8,6,BLUE);
            displayColor(8,5,BLUE);
            displayColor(8,4,BLUE);
            displayColor(8,3,BLUE);
            displayColor(9,7,BLUE);
            displayColor(9,6,BLUE);
            displayColor(9,5,BLUE);
            displayColor(9,4,BLUE);
            displayColor(9,3,BLUE);
            displayColor(10,7,BLUE);
            displayColor(10,6,BLUE);
            displayColor(10,5,BLUE);
            displayColor(10,4,BLUE);
            displayColor(10,3,BLUE);
            displayColor(10,2,BLUE);
            displayColor(11,7,BLUE);
            displayColor(11,6,BLUE);
            displayColor(11,5,BLUE);
            displayColor(11,4,BLUE);
            displayColor(11,3,BLUE);
            displayColor(11,2,BLUE);
            displayColor(12,7,WHITE);
            displayColor(12,6,WHITE);
            displayColor(12,5,WHITE);
            displayColor(12,4,WHITE);
            displayColor(12,3,WHITE);
            displayColor(12,2,WHITE);
            displayColor(12,1,WHITE);
            displayColor(13,7,WHITE);
            displayColor(13,6,WHITE);
            displayColor(13,5,WHITE);
            displayColor(13,4,WHITE);
            displayColor(13,3,WHITE);
            displayColor(13,2,WHITE);
            displayColor(13,1,WHITE);
            displayColor(14,7,WHITE);
            displayColor(14,6,WHITE);
            displayColor(14,5,WHITE);
            displayColor(14,4,WHITE);
            displayColor(14,3,WHITE);
            displayColor(14,2,WHITE);
            displayColor(14,1,WHITE);
            displayColor(14,0,WHITE);
            displayColor(15,7,WHITE);
            displayColor(15,6,WHITE);
            displayColor(15,5,WHITE);
            displayColor(15,4,WHITE);
            displayColor(15,3,WHITE);
            displayColor(15,2,WHITE);
            displayColor(15,1,WHITE);
            displayColor(15,0,WHITE);
        } else if(digital_emg_signal_value >= 3072 && digital_emg_signal_value <= 3327) {
            displayColor(0,7,RED);
            displayColor(1,7,RED);
            displayColor(2,7,RED);
            displayColor(2,6,RED);
            displayColor(3,7,RED);
            displayColor(3,6,RED);
            displayColor(4,7,YELLOW);
            displayColor(4,6,YELLOW);
            displayColor(4,5,YELLOW);
            displayColor(5,7,YELLOW);
            displayColor(5,6,YELLOW);
            displayColor(5,5,YELLOW);
            displayColor(6,7,YELLOW);
            displayColor(6,6,YELLOW);
            displayColor(6,5,YELLOW);
            displayColor(6,4,YELLOW);
            displayColor(7,7,YELLOW);
            displayColor(7,6,YELLOW);
            displayColor(7,5,YELLOW);
            displayColor(7,4,YELLOW);
            displayColor(8,7,BLUE);
            displayColor(8,6,BLUE);
            displayColor(8,5,BLUE);
            displayColor(8,4,BLUE);
            displayColor(8,3,BLUE);
            displayColor(9,7,BLUE);
            displayColor(9,6,BLUE);
            displayColor(9,5,BLUE);
            displayColor(9,4,BLUE);
            displayColor(9,3,BLUE);
            displayColor(10,7,BLUE);
            displayColor(10,6,BLUE);
            displayColor(10,5,BLUE);
            displayColor(10,4,BLUE);
            displayColor(10,3,BLUE);
            displayColor(10,2,BLUE);
            displayColor(11,7,BLUE);
            displayColor(11,6,BLUE);
            displayColor(11,5,BLUE);
            displayColor(11,4,BLUE);
            displayColor(11,3,BLUE);
            displayColor(11,2,BLUE);
            displayColor(12,7,GREEN);
            displayColor(12,6,GREEN);
            displayColor(12,5,GREEN);
            displayColor(12,4,GREEN);
            displayColor(12,3,GREEN);
            displayColor(12,2,GREEN);
            displayColor(12,1,GREEN);
            displayColor(13,7,WHITE);
            displayColor(13,6,WHITE);
            displayColor(13,5,WHITE);
            displayColor(13,4,WHITE);
            displayColor(13,3,WHITE);
            displayColor(13,2,WHITE);
            displayColor(13,1,WHITE);
            displayColor(14,7,WHITE);
            displayColor(14,6,WHITE);
            displayColor(14,5,WHITE);
            displayColor(14,4,WHITE);
            displayColor(14,3,WHITE);
            displayColor(14,2,WHITE);
            displayColor(14,1,WHITE);
            displayColor(14,0,WHITE);
            displayColor(15,7,WHITE);
            displayColor(15,6,WHITE);
            displayColor(15,5,WHITE);
            displayColor(15,4,WHITE);
            displayColor(15,3,WHITE);
            displayColor(15,2,WHITE);
            displayColor(15,1,WHITE);
            displayColor(15,0,WHITE);
        } else if(digital_emg_signal_value >= 3328 && digital_emg_signal_value <= 3583) {
            displayColor(0,7,RED);
            displayColor(1,7,RED);
            displayColor(2,7,RED);
            displayColor(2,6,RED);
            displayColor(3,7,RED);
            displayColor(3,6,RED);
            displayColor(4,7,YELLOW);
            displayColor(4,6,YELLOW);
            displayColor(4,5,YELLOW);
            displayColor(5,7,YELLOW);
            displayColor(5,6,YELLOW);
            displayColor(5,5,YELLOW);
            displayColor(6,7,YELLOW);
            displayColor(6,6,YELLOW);
            displayColor(6,5,YELLOW);
            displayColor(6,4,YELLOW);
            displayColor(7,7,YELLOW);
            displayColor(7,6,YELLOW);
            displayColor(7,5,YELLOW);
            displayColor(7,4,YELLOW);
            displayColor(8,7,BLUE);
            displayColor(8,6,BLUE);
            displayColor(8,5,BLUE);
            displayColor(8,4,BLUE);
            displayColor(8,3,BLUE);
            displayColor(9,7,BLUE);
            displayColor(9,6,BLUE);
            displayColor(9,5,BLUE);
            displayColor(9,4,BLUE);
            displayColor(9,3,BLUE);
            displayColor(10,7,BLUE);
            displayColor(10,6,BLUE);
            displayColor(10,5,BLUE);
            displayColor(10,4,BLUE);
            displayColor(10,3,BLUE);
            displayColor(10,2,BLUE);
            displayColor(11,7,BLUE);
            displayColor(11,6,BLUE);
            displayColor(11,5,BLUE);
            displayColor(11,4,BLUE);
            displayColor(11,3,BLUE);
            displayColor(11,2,BLUE);
            displayColor(12,7,GREEN);
            displayColor(12,6,GREEN);
            displayColor(12,5,GREEN);
            displayColor(12,4,GREEN);
            displayColor(12,3,GREEN);
            displayColor(12,2,GREEN);
            displayColor(12,1,GREEN);
            displayColor(13,7,GREEN);
            displayColor(13,6,GREEN);
            displayColor(13,5,GREEN);
            displayColor(13,4,GREEN);
            displayColor(13,3,GREEN);
            displayColor(13,2,GREEN);
            displayColor(13,1,GREEN);
            displayColor(14,7,WHITE);
            displayColor(14,6,WHITE);
            displayColor(14,5,WHITE);
            displayColor(14,4,WHITE);
            displayColor(14,3,WHITE);
            displayColor(14,2,WHITE);
            displayColor(14,1,WHITE);
            displayColor(14,0,WHITE);
            displayColor(15,7,WHITE);
            displayColor(15,6,WHITE);
            displayColor(15,5,WHITE);
            displayColor(15,4,WHITE);
            displayColor(15,3,WHITE);
            displayColor(15,2,WHITE);
            displayColor(15,1,WHITE);
            displayColor(15,0,WHITE);
        } else if(digital_emg_signal_value >= 3584 && digital_emg_signal_value <= 3839){
            displayColor(0,7,RED);
            displayColor(1,7,RED);
            displayColor(2,7,RED);
            displayColor(2,6,RED);
            displayColor(3,7,RED);
            displayColor(3,6,RED);
            displayColor(4,7,YELLOW);
            displayColor(4,6,YELLOW);
            displayColor(4,5,YELLOW);
            displayColor(5,7,YELLOW);
            displayColor(5,6,YELLOW);
            displayColor(5,5,YELLOW);
            displayColor(6,7,YELLOW);
            displayColor(6,6,YELLOW);
            displayColor(6,5,YELLOW);
            displayColor(6,4,YELLOW);
            displayColor(7,7,YELLOW);
            displayColor(7,6,YELLOW);
            displayColor(7,5,YELLOW);
            displayColor(7,4,YELLOW);
            displayColor(8,7,BLUE);
            displayColor(8,6,BLUE);
            displayColor(8,5,BLUE);
            displayColor(8,4,BLUE);
            displayColor(8,3,BLUE);
            displayColor(9,7,BLUE);
            displayColor(9,6,BLUE);
            displayColor(9,5,BLUE);
            displayColor(9,4,BLUE);
            displayColor(9,3,BLUE);
            displayColor(10,7,BLUE);
            displayColor(10,6,BLUE);
            displayColor(10,5,BLUE);
            displayColor(10,4,BLUE);
            displayColor(10,3,BLUE);
            displayColor(10,2,BLUE);
            displayColor(11,7,BLUE);
            displayColor(11,6,BLUE);
            displayColor(11,5,BLUE);
            displayColor(11,4,BLUE);
            displayColor(11,3,BLUE);
            displayColor(11,2,BLUE);
            displayColor(12,7,GREEN);
            displayColor(12,6,GREEN);
            displayColor(12,5,GREEN);
            displayColor(12,4,GREEN);
            displayColor(12,3,GREEN);
            displayColor(12,2,GREEN);
            displayColor(12,1,GREEN);
            displayColor(13,7,GREEN);
            displayColor(13,6,GREEN);
            displayColor(13,5,GREEN);
            displayColor(13,4,GREEN);
            displayColor(13,3,GREEN);
            displayColor(13,2,GREEN);
            displayColor(13,1,GREEN);
            displayColor(14,7,GREEN);
            displayColor(14,6,GREEN);
            displayColor(14,5,GREEN);
            displayColor(14,4,GREEN);
            displayColor(14,3,GREEN);
            displayColor(14,2,GREEN);
            displayColor(14,1,GREEN);
            displayColor(14,0,GREEN);
            displayColor(15,7,WHITE);
            displayColor(15,6,WHITE);
            displayColor(15,5,WHITE);
            displayColor(15,4,WHITE);
            displayColor(15,3,WHITE);
            displayColor(15,2,WHITE);
            displayColor(15,1,WHITE);
            displayColor(15,0,WHITE);
        } else if(digital_emg_signal_value >= 3840 && digital_emg_signal_value <= 4095) {
            displayColor(0,7,RED);
            displayColor(1,7,RED);
            displayColor(2,7,RED);
            displayColor(2,6,RED);
            displayColor(3,7,RED);
            displayColor(3,6,RED);
            displayColor(4,7,YELLOW);
            displayColor(4,6,YELLOW);
            displayColor(4,5,YELLOW);
            displayColor(5,7,YELLOW);
            displayColor(5,6,YELLOW);
            displayColor(5,5,YELLOW);
            displayColor(6,7,YELLOW);
            displayColor(6,6,YELLOW);
            displayColor(6,5,YELLOW);
            displayColor(6,4,YELLOW);
            displayColor(7,7,YELLOW);
            displayColor(7,6,YELLOW);
            displayColor(7,5,YELLOW);
            displayColor(7,4,YELLOW);
            displayColor(8,7,BLUE);
            displayColor(8,6,BLUE);
            displayColor(8,5,BLUE);
            displayColor(8,4,BLUE);
            displayColor(8,3,BLUE);
            displayColor(9,7,BLUE);
            displayColor(9,6,BLUE);
            displayColor(9,5,BLUE);
            displayColor(9,4,BLUE);
            displayColor(9,3,BLUE);
            displayColor(10,7,BLUE);
            displayColor(10,6,BLUE);
            displayColor(10,5,BLUE);
            displayColor(10,4,BLUE);
            displayColor(10,3,BLUE);
            displayColor(10,2,BLUE);
            displayColor(11,7,BLUE);
            displayColor(11,6,BLUE);
            displayColor(11,5,BLUE);
            displayColor(11,4,BLUE);
            displayColor(11,3,BLUE);
            displayColor(11,2,BLUE);
            displayColor(12,7,GREEN);
            displayColor(12,6,GREEN);
            displayColor(12,5,GREEN);
            displayColor(12,4,GREEN);
            displayColor(12,3,GREEN);
            displayColor(12,2,GREEN);
            displayColor(12,1,GREEN);
            displayColor(13,7,GREEN);
            displayColor(13,6,GREEN);
            displayColor(13,5,GREEN);
            displayColor(13,4,GREEN);
            displayColor(13,3,GREEN);
            displayColor(13,2,GREEN);
            displayColor(13,1,GREEN);
            displayColor(14,7,GREEN);
            displayColor(14,6,GREEN);
            displayColor(14,5,GREEN);
            displayColor(14,4,GREEN);
            displayColor(14,3,GREEN);
            displayColor(14,2,GREEN);
            displayColor(14,1,GREEN);
            displayColor(14,0,GREEN);
            displayColor(15,7,GREEN);
            displayColor(15,6,GREEN);
            displayColor(15,5,GREEN);
            displayColor(15,4,GREEN);
            displayColor(15,3,GREEN);
            displayColor(15,2,GREEN);
            displayColor(15,1,GREEN);
            displayColor(15,0,GREEN);
        }
    }
    
    return 1; 
}