package org.usfirst.frc.team3216.robot;

import edu.wpi.first.wpilibj.*;


/**
 * @author jacksonservheen
 *
 */
public class DigitBoard {
	I2C comm;
	
	DigitBoard() {
		this.comm = new I2C(I2C.Port.kMXP, 0x70);
		//this.turnOn();
	}
	
	void turnOn() {
		this.comm.write(0x80,129);
	}
	void turnOff() {
		this.comm.write(0x80,128);
	}
	
	void setDigit(int digit, int value) {
		this.comm.write(digit*2,value&255);
		this.comm.write(digit*2+1,(value>>8)&255);
	}
}
