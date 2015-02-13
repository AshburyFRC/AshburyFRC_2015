package org.usfirst.frc4825.FRC_2015.subsystems;

import java.util.Comparator;
import java.util.Vector;

import org.usfirst.frc4825.FRC_2015.Robot;
import org.usfirst.frc4825.FRC_2015.RobotMap;
import org.usfirst.frc4825.FRC_2015.commands.*;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ImageType;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Ultra Pro Code By Isidor Ehrlich
 */
public class DriveTrain extends Subsystem {
	SpeedController leftSpeedController = RobotMap.driveTrainLeftSpeedController;
	SpeedController rightSpeedController = RobotMap.driveTrainRightSpeedController;
	RobotDrive robotDrive21 = RobotMap.driveTrainRobotDrive21;
	DigitalInput driveSwitch = RobotMap.driveTrainDriveSwitch;
	Encoder encoder = RobotMap.driveTrainEncoder;
	Gyro gyro = RobotMap.driveTrainGyro;
	double angle = gyro.getAngle();
	int session = 0;
	
	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new DriveWithJoystick());
	}

	public void driveStraitNoSensor(double speed) {
		// Drive in a strait line at a given speed; negative is reverse
		robotDrive21.drive(speed, 0.0);
	}

	public void driveStraitSensor(double speed) {
		// Drive in a strait line at a given speed; negative is reverse; uses gyro
		double angle = RobotMap.driveTrainGyro.getAngle(); // get current
															// heading
		robotDrive21.drive(speed, -angle * 0.03); // drive towards heading 0
		Timer.delay(0.004);
	}

	public void stop() {
		// Stop the drive train
		robotDrive21.drive(0.0, 0.0);
	}

	public void processJoystickInput(Joystick stick1) {
		// arcade drive
		int signX = (stick1.getRawAxis(4) > 1) ? 1 : -1;
		double speed = Math.pow(stick1.getAxis(Joystick.AxisType.kY), 3);
		double rotation = (Math.pow(stick1.getRawAxis(4), 5) * signX);
		if (rotation < 0.5 && rotation > -0.5) {// ensures the robot goes straight within the specified dead zone using a gyro
			Robot.driveTrain.driveStraitSensor(speed);
		} else {
			robotDrive21.arcadeDrive(speed, rotation);
		}
	}

	public boolean atSwitch() {
		// Returns if limit switch is pressed
		return Robot.driveTrain.driveSwitch.get();
	}

	public boolean turn90DegreesLeft() {
		// Turns the robot to the right 90 degrees
		robotDrive21.tankDrive(-0.8, 0.8);
		if (gyro.getAngle() <= angle - 90) {
			return true;
		} else {
			return false;
		}
	}

	public void resetTurnAngle() {
		// Resets the saved starting angle and gyro
		gyro.reset();
		angle = gyro.getAngle();
	}
	
	
	
	//\\ Image Processing //\\
}