package org.usfirst.frc.team3216.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

import org.usfirst.frc.team3216.robot.DigitBoard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

/**********
 * Controls overview:
 * left joystick - forward and backward controls forward speed, left and right controls strafe
 * right joystick - left and right controls rotation
 * triggers - difference between triggers controls winch
 * bumpers - left bumper opens claw
 *		 - right bumper closes claw
 * 
 * 
 * @author jacksonservheen
 *
 */

/********
 * Connections:
 * Talons: front left = pwm 2
 *		 front right = pwm 0
 *		 back left = pwm 1
 *		 back right = pwm 3
 * Winch spikes = pwm 4
 * Limit switches: bottom = dio 0
 *				 top = dio 1
 * Claw doublesolenoid = solenoid 0&1
 * winch stop doublesolenoid = solenoid 2&3
 * 
 * @author jacksonservheen
 *
 */

public class Robot extends IterativeRobot {
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	// global objects:
	RobotDrive mecanum;
	
	Joystick xBox; // xbox controller, obvoiusly
	
	Timer clawTimer; // claw timing
	
	Compressor pcm; // pneumatics compressor
	Talon winch; // talon for controlling winch
	DoubleSolenoid winchstop;
	DoubleSolenoid claw; // double solenoid for claw
	DigitalInput blimitsw,tlimitsw; // limit switches
	//Gyro gyro;
	AnalogInput range;
	
	Encoder winchenc;
	BuiltInAccelerometer acc;
	NetworkTable table;
	
	PowerDistributionPanel pdp;
	DriverStation ds;
	
	DigitalOutput alliance1,alliance2,enabledp;
	
	DigitBoard db;
	
	int a;

	
	
	public void robotInit() {
		// wpilibj uses zero-based indices for components this year, 
		// but last year wpilibc use 1-based indices (including joysticks)
		clawTimer = new Timer();
		xBox	  = new Joystick(0);
		
		pcm = new Compressor(0);
		claw = new DoubleSolenoid(0,1); // easiest way to reverse the controls for the claw is to switch these numbers
		winch = new Talon(4);
		winchstop = new DoubleSolenoid(3,2);
		blimitsw = new DigitalInput(0);
		tlimitsw = new DigitalInput(1);
		//gyro = new Gyro(0);
		range = new AnalogInput(1);
		
		winchenc = new Encoder(3,4);
		winchenc.setDistancePerPulse(1/360.0); 
		
		acc = new BuiltInAccelerometer();
		table = NetworkTable.getTable("datatable");
		
		pdp = new PowerDistributionPanel();
		
		mecanum = new RobotDrive(1,0,2,3);
		
		alliance1 = new DigitalOutput(11); // red
		alliance2 = new DigitalOutput(12); // blue
		enabledp = new DigitalOutput(13); // enabled
		
		ds = DriverStation.getInstance();
		
		pcm.setClosedLoopControl(true); // this is now done programatically by the pneumatics module
		for (int i = 0; i < numreadings; i++) readings[i] = initvalue;
		
		autonstart = false;
		autonend = false;
		
		db = new DigitBoard();
		
		a = 0;
	}
	
	public void disabledPeriodic() {
		sendData();
		
		db.setDigit(0,a);
		a++;
		a%=65535;
	}

	/**
	 * This function is called periodically during autonomous
	 */
	// auton smoothing
	public final static int initvalue = 250;
	public final static int numreadings = 3;          //The number of samples to smooth.  Helps to avoid incorrect readings due to noise or voltage spikes  
	public static double[] readings = new double[numreadings];  //Stores the values from the rangefinder
	public static int index   = 0;              //The location within the array to store the value
	public static double total = initvalue*numreadings;
	public static double r_average = 0;  
	public static double a_range;
	
	public final static int setpoint = 250; // centimeters (411)
	
	static boolean autonstart = false, autonend = false;
	
	public void autonomousPeriodic() {
		sendData(); // this also does the moving average stuff
		
		if (!autonstart) {
			mecanum.mecanumDrive_Cartesian(0,0,0,0);
			Timer.delay(0.5);
			mecanum.mecanumDrive_Cartesian(0,0,0,0);
			manageClaw(-1);
			manageWinch2(-0.2);
			Timer.delay(0.5);
			manageWinch2(0);
			
			autonstart = true;
		} else if (autonstart && !autonend) {
			double speed = 0.5; //map(a_range, 0, 200, 0, 1)*0.5+0.1;
			
			if (r_average < setpoint) {
				mecanum.mecanumDrive_Cartesian(0,0,speed,0);
			} else {
				autonend = true;
			}
		} else if (autonend) {
			mecanum.mecanumDrive_Cartesian(0,0,0,0);
		}
		
	}

	double map(double value, double istart, double istop, double ostart, double ostop){
		return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
	}

	/**
	 * This function is called periodically during operator control
	 */
	// variables used in teleop:
	double rotate, triggers, forward, strafe;
	
	public void teleopPeriodic() {
		// this took a lot of guess-and-check, since several 
		// axes were messed up this year. don't trust the 
		// joystick explorer, use the DS for testing these
		rotate = -xBox.getRawAxis(4);
		forward = -xBox.getRawAxis(1);	
		strafe = xBox.getRawAxis(0);
			
		mecanum.mecanumDrive_Cartesian(strafe*0.60, Math.pow(rotate,3)*0.40, forward*0.60, 0); // 0 for non-field centric
		//mecanum.mecanumDrive_Cartesian(-strafe*0.75, rotate*0.4, -forward*0.75, 0);
		
		// manage the claw - bumpers
		int clawdir = 0;
		boolean open = xBox.getRawButton(6), close = xBox.getRawButton(5); //get button state
		if (open) clawdir = 1; // in/out logic
		if (close) clawdir = -1;
		manageClaw(clawdir); // manage
		
		// manage the winch 
		// the triggers last year were read as a single value,
		// but this year they are separate
		triggers = xBox.getRawAxis(2) - xBox.getRawAxis(3);	
		//boolean up = xBox.getRawButton(6), down = xBox.getRawButton(5); // get bumper state
		boolean blimit = blimitsw.get(), tlimit = tlimitsw.get(); // get limit switch state
		if (!blimit && triggers < 0) triggers = 0; // stop for the limit switch
		if (!tlimit && triggers > 0) triggers = 0;
		//if (!blimit && winchdir == -1) winchstopped = true; // stop for the limit switch
		//if (!tlimit && winchdir == 1) winchstopped = true;
		manageWinch2(triggers*0.45); // manage
		
		sendData();
	}
	
	void manageClaw(int direction) {
		// pass 1 to open
		//	 -1 to close
		//	  0 to do nothing
		
		// this will stop the solenoid 30 msec after no buttons are pressed
		if (direction == 0 && clawTimer.get() > 0.3) {  //TODO: fix this logic -- i think its broken
			claw.set(DoubleSolenoid.Value.kOff);
			clawTimer.stop();
			clawTimer.reset();
		}
		else if (direction == 1) {
			claw.set(DoubleSolenoid.Value.kForward);
			clawTimer.start();
		}
		else if (direction == -1) {
			claw.set(DoubleSolenoid.Value.kReverse);
			clawTimer.start();
		}
	}
	
	double winchdelay = 0.06;
	boolean winchstopped = false;
	
	void manageWinch2(double trig) {
		if (Math.abs(trig) < 0.1 && !winchstopped) { 
			winch.set(0);
			winchstop.set(DoubleSolenoid.Value.kForward);
			Timer.delay(winchdelay);
			winchstop.set(DoubleSolenoid.Value.kOff);
			winchstopped = true;
		}
		else if (Math.abs(trig) > 0.1) {
			if (winchstopped) {
				winchstop.set(DoubleSolenoid.Value.kReverse);
				Timer.delay(winchdelay);
				winchstop.set(DoubleSolenoid.Value.kOff);
				winchstopped = false;
			}
			winch.set(trig);
		}
	}
	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		// no testing period
		sendData();
	}
	
	byte[] lastmode = new byte[10];
	
	void sendData() {
		//moving average for rangefinder
		double a_range = range.getVoltage() / (5.0/1024.0); // centimeters
		total -= readings[index];         //subtract the last reading
		readings[index] = a_range;        //place value from sensor
		total += readings[index];         //add the reading to the total
		index++;                          //advance to the next position in the array
		if (index>=numreadings) index=0;  //if its at the end of the array, wrap around to the beginning
		r_average = total / numreadings;    //calculate the average
		
		// put data into table
		table.putNumber("accel_x",acc.getX());
		table.putNumber("accel_y",acc.getY());
		table.putNumber("accel_z",acc.getZ());
		//table.putNumber("gyro_r",gyro.getAngle());
		//table.putNumber("gyro_s",gyro.getRate());
		table.putNumber("pwr_v",pdp.getVoltage());
		table.putNumber("pwr_t",pdp.getTemperature());
		table.putNumber("pwr_c",pdp.getTotalCurrent());
		for (int i = 0; i < 16; i++) 
			table.putNumber("pwr_c_" + i,pdp.getCurrent(i));
		table.putNumber("pcm_c",pcm.getCompressorCurrent());
		table.putNumber("enc_r",winchenc.getRate());
		table.putNumber("range",a_range);
		
		// now do arduino stuff
		byte mode1[] = new byte[10];
		if (ds.getAlliance() == DriverStation.Alliance.Red) mode1[1] = 1;
		if (ds.getAlliance() == DriverStation.Alliance.Blue) mode1[1] = 2;
		if (ds.isFMSAttached()) mode1[3] = 1;
		if (ds.isAutonomous()) mode1[2] = 3;
		if (ds.isDisabled()) {mode1[2] = 1; mode1[4] = 0;}
		if (ds.isOperatorControl()) mode1[2] = 2;
		if (ds.isTest()) mode1[2] = 4;
		if (ds.isEnabled()) mode1[4] = 1;
		
		if (mode1[1] == 1) {
			alliance1.set(true);
			alliance2.set(false);
		} else if (mode1[1] == 2) {
			alliance2.set(true);
			alliance1.set(false);
		} else {
			alliance1.set(false);
			alliance2.set(false);
		}
		if (mode1[4] == 1) {
			enabledp.set(true);
		} else {
			enabledp.set(false);
		}
		
		
	}
	/*
	serial protocol:
	each message starts with !
	if one side is out of sync, it will send a ? character to restart the last transmission
	message ends with an #
	rio will send byte message in the form of !<alliance><mode1><fms><enabled><mode2>#
	alliance - ascii digit 0,1,2
	  0 = no alliance, make lights flash
	  1 = red alliance
	  2 = blue alliance
	mode1 - auton/teleop
	  0 = default, flash lights (no DS)
	  1 = disabled
	  2 = teleop
	  3 = auton
	  4 = test
	fms - on the field (0,1)
	enabled - enabled (0,1)
	mode2 - light pattern
	*/
}
