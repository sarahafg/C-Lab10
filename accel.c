/*--------------------------------------------------------
GEORGE MASON UNIVERSITY
ECE 447 - Accelerometer routines
Date:   Fall 2020
Author: Jens-Peter Kaps, Sarah Fakhry
--------------------------------------------------------*/

#include "accel.h"
#include <msp430.h>
#include <stdio.h>
#include <stdint.h>
#include "lcd.h"
#include "graphics.h"
#include "color.h"
#include "ports.h"

#define MAX 1
int16_t coor_x = 0;
int16_t coor_y = 0;
uint8_t rx_val = 0;
char x[15];
char y[15];
char incoming[MAX] = {0};
uint8_t sending[MAX] = {0};
uint8_t b_rx = 0;
uint8_t b_tx = 0;
uint8_t temp = 0;
uint8_t temp2 = 0;
uint8_t only_b = 0;
float val_x = 0;
float val_y = 0;
uint8_t address_tx = 0;
uint8_t address_rx = 0;

typedef enum I2C_Enum{
    IDLE_MODE,
    NACK_MODE,
    TX_REG_ADDRESS_MODE,
    RX_REG_ADDRESS_MODE,
    TX_DATA_MODE,
    RX_DATA_MODE,
    SWITCH_TO_RX_MODE,
    SWITHC_TO_TX_MODE,
    TIMEOUT_MODE
} I2C_Mode;

I2C_Mode track = IDLE_MODE;

void writeData(uint8_t data) {
    P2OUT &= ~LCD_CS_PIN;
    P4OUT |= LCD_DC_PIN;
    UCB0TXBUF = data;
}

void writeCommand(uint8_t command) {
    P2OUT &= ~LCD_CS_PIN;
    P4OUT &= ~LCD_DC_PIN;
    UCB0TXBUF = command;

}

void introScreen(void) {
    clearScreen(10);
    setColor(COLOR_16_GREEN);
    drawString(30,50, FONT_LG, "MAZE GAME");

}

int accel_setup() {
	// Initialize the sensor
    // set power mode and bandwidth
    // set data rate, enable X and Y axis
    // set range and resolution
    track = TX_REG_ADDRESS_MODE;
    address_tx = MSA301_REG_PARTID;
    only_b = 1;
    b_tx = 1;
    b_rx = 1;
    temp = 0;
    temp2 = 0;
    UCB1I2CSA = MSA301_I2CADDR_DEFAULT;
    UCB1IFG &= ~(UCTXIFG0 + UCRXIFG0);
    UCB1IE &= ~UCRXIE0;
    UCB1IE |= UCTXIE0;
    UCB1CTLW0 |= UCTR + UCTXSTT;
    __delay_cycles(30);
    track = TX_REG_ADDRESS_MODE;
    address_tx = MSA301_REG_POWERMODE;
    b_tx = 1;
    b_rx = 0;
    temp = 0;
    temp2 = 0;
    UCB1I2CSA = MSA301_I2CADDR_DEFAULT;
    UCB1IFG &= ~(UCTXIFG0 + UCRXIFG0);
    UCB1IE &= ~UCRXIE0;
    UCB1IE |= UCTXIE0;
    sending[0] = MSA301_BANDWIDTH_250_HZ;
    UCB1CTLW0 |= UCTR + UCTXSTT;
    __delay_cycles(30);
    track = TX_REG_ADDRESS_MODE;
    address_tx = MSA301_REG_ODR;
    b_tx = 1;
    b_rx = 0;
    temp = 0;
    temp2 = 0;
    UCB1I2CSA = MSA301_I2CADDR_DEFAULT;
    UCB1IFG &= ~(UCTXIFG0 + UCRXIFG0);
    UCB1IE &= ~UCRXIE0;
    UCB1IE |= UCTXIE0;
    sending[0] = MSA301_DATARATE_500_HZ;
    UCB1CTLW0 |= UCTR + UCTXSTT;
    __delay_cycles(30);
    track = TX_REG_ADDRESS_MODE;
    address_rx = MSA301_REG_RESRANGE;
    b_tx = 1;
    b_rx = 0;
    temp = 0;
    temp2 = 0;
    UCB1I2CSA = MSA301_I2CADDR_DEFAULT;
    UCB1IFG &= ~(UCTXIFG0 + UCRXIFG0);
    UCB1IE &= ~UCRXIE0;
    UCB1IE |= UCTXIE0;
    sending[0] = MSA301_RESOLUTION_14;
    UCB1CTLW0 |= UCTR + UCTXSTT;
    return 0;
}

I2C_Mode read_help(uint8_t address) {
    track = TX_REG_ADDRESS_MODE;
    address_tx = address;
    only_b = 1;
    b_tx = 1;
    b_rx = 1;
    temp = 0;
    temp2 = 0;
    UCB1I2CSA = MSA301_I2CADDR_DEFAULT;
    UCB1IFG &= ~(UCTXIFG0 + UCRXIFG0);
    UCB1IE &= ~UCRXIE0;
    UCB1IE |= UCTXIE0;
    UCB1CTLW0 |= UCTR + UCTXSTT;
    return track;
}

void accel_update() {
    // X LSB
    // X MSB
    // Y LSB
    // Y MSB
    read_help(MSA301_REG_OUT_X_L);
    __delay_cycles(30);
    coor_x = incoming[0];
    read_help(MSA301_REG_OUT_X_H);
    __delay_cycles(30);
    coor_x = ((incoming[0] << 8) | coor_x) / 4;
    read_help(MSA301_REG_OUT_Y_L);
    __delay_cycles(30);
    coor_y = incoming[0];
    read_help(MSA301_REG_OUT_Y_H);
    __delay_cycles(30);
    coor_y = ((incoming[0] << 8) | coor_y) / 4;
    val_x = (float) coor_x / 2048;
    val_y = (float) coor_y / 2048;
    sprintf(x, "X: %f", val_x);
    sprintf(y, "Y: %f", val_y);
}

void main(void) {
    WDTCTL = WDTPW | WDTHOLD;   // kill the watchdog
    P1DIR   |= BIT3;
    P1SEL0  |= BIT3;
    P1SEL1  &= ~BIT3;
    TA1CCR0  = 511;
    TA1CCTL2 = OUTMOD_7;
    TA1CCR2  = 255;
    TA1CTL   = TASSEL__ACLK | MC__UP | TACLR;
    P2DIR  |=   LCD_CS_PIN;
    P4DIR  |=   LCD_DC_PIN;
    P1SEL0 |=   LCD_MOSI_PIN | LCD_UCBCLK_PIN;
    P1SEL1 &= ~(LCD_MOSI_PIN | LCD_UCBCLK_PIN);
    UCB0CTLW0 |= UCSWRST;
    UCB0CTLW0 |= UCSSEL__SMCLK | UCCKPH | UCMSB | UCMST | UCMODE_0 | UCSYNC;
    UCB0BR0   |= 0x01;
    UCB0BR1    = 0;
    UCB0CTL1  &= ~UCSWRST;
    PM5CTL0   &= ~LOCKLPM5;
    P4SEL0 &= ~(BIT0 | BIT1);
    P4SEL1 |= (BIT0 | BIT1);
    UCB1CTL1 |= UCSWRST;
    UCB1CTLW0 |= UCMODE_3 + UCMST + UCSSEL__SMCLK;
    UCB1BRW = 0x0003;
    UCB1I2CSA = MSA301_I2CADDR_DEFAULT;
    UCB1CTL1 &= ~UCSWRST;
    __enable_interrupt();
    __delay_cycles(10);
    accel_setup();
    initLCD();
    introScreen();
    while (TRUE) {
        accel_update();
        clearScreen(10);
        drawCircle(10, 5, 5);
        drawString(15,40, FONT_MD, x);
        drawString(15,70, FONT_MD, y);
        _nop();
    }
}

#pragma vector = USCI_B1_VECTOR
__interrupt void USCI_B1_ISR(void) {
    switch(__even_in_range(UCB1IV,USCI_I2C_UCBIT9IFG)) {
    case 0x16:      // Vector 22: RXIFG0
        rx_val = UCB1RXBUF;
        if (b_rx == 0) {
            UCB1IE &= ~UCRXIE;
            track = IDLE_MODE;
        }
        if (b_rx) {
            incoming[temp++] = rx_val;
            b_rx--;
        }
        if (b_rx == 1) {
            UCB1CTLW0 |= UCTXSTP;
        }
        break;
    case 0x18:      // Vector 24: TXIFG0
        switch(track) {
        case TX_REG_ADDRESS_MODE:
            UCB1TXBUF = address_tx;
            if (b_rx) {
                track = SWITCH_TO_RX_MODE;
            }
            else {
                track = TX_DATA_MODE;
            }
            break;
        case TX_DATA_MODE:
            if (b_tx) {
                UCB1TXBUF = sending[temp2++];
                b_tx--;
            }
            else  {
                UCB1CTLW0 |= UCTXSTP;
                track = IDLE_MODE;
                UCB1IE &= ~UCTXIE;
            }
            break;
        case SWITCH_TO_RX_MODE:
            UCB1IE |= UCRXIE;
            UCB1IE &= ~UCTXIE;
            UCB1CTLW0 &= ~UCTR;
            track = RX_DATA_MODE;
            UCB1CTLW0 |= UCTXSTT;
            if (b_rx == 1) {
                while((UCB1CTLW0 & UCTXSTT));
                UCB1CTLW0 |= UCTXSTP;
            }
            break;
            default:
                break;
        }
        break;
        default: break;
    }
}
