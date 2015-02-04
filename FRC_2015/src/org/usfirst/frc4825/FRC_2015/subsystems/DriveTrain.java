package org.usfirst.frc4825.FRC_2015.subsystems;

import org.usfirst.frc4825.FRC_2015.RobotMap;
import org.usfirst.frc4825.FRC_2015.commands.*;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *Ultra Pro Code By Isidor Ehrlich
 */
public class DriveTrain extends Subsystem {
    SpeedController leftSpeedController = RobotMap.driveTrainLeftSpeedController;
    SpeedController rightSpeedController = RobotMap.driveTrainRightSpeedController;
    RobotDrive robotDrive21 = RobotMap.driveTrainRobotDrive21;
    DigitalInput driveSwitch = RobotMap.driveTrainDriveSwitch;
    Encoder encoder = RobotMap.driveTrainEncoder;
    Gyro gyro = RobotMap.driveTrainGyro;
    private final double DRIFT = -0.1;
    private final double MOTOR_SMOOTHING_OMEGA = 1.0;

    
    // Put methods for controlling this subsystem here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
    	setDefaultCommand(new DriveWithJoystick());
    }
    
    public void driveStraitNoSensor(double speed){
    	//Drive in a strait line at a given speed; negative is reverse
    	robotDrive21.drive(speed, 0.0);
    }
    
    public void driveStraitSensor(double speed){
    	//Drive in a strait line at a given speed; negative is reverse; uses gyro
    	double angle = RobotMap.driveTrainGyro.getAngle(); // get current heading
        System.out.println(angle);
        robotDrive21.drive(speed, DRIFT + angle * 0.03); // drive towards heading 0
        Timer.delay(0.004);
    }
    
    public void stop(){
    	//Stop the drive train
    	robotDrive21.drive(0.0, 0.0);
    }
    
    public void processJoystickInput(Joystick stick1) {
        //arcade drive
    	int signX = (stick1.getRawAxis(4) > 1) ? 1 : -1;
    	double speed = Math.pow(stick1.getAxis(Joystick.AxisType.kY), 3);
    	double rotation = (Math.pow(stick1.getRawAxis(4), 5)*signX);
        robotDrive21.arcadeDrive(speed,rotation, false);
        System.out.println(rotation);
    }
}

