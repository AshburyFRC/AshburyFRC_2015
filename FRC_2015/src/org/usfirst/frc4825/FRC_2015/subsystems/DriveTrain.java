// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc4825.FRC_2015.subsystems;

import org.usfirst.frc4825.FRC_2015.RobotMap;
import org.usfirst.frc4825.FRC_2015.commands.*;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.CounterBase.EncodingType; import edu.wpi.first.wpilibj.PIDSource.PIDSourceParameter;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 *
 */
public class DriveTrain extends Subsystem {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    SpeedController leftDriveController = RobotMap.driveTrainLeftDriveController;
    SpeedController rightDriveController = RobotMap.driveTrainRightDriveController;
    RobotDrive robotDrive = RobotMap.driveTrainRobotDrive;
    DigitalInput driveLimitSwitch = RobotMap.driveTrainDriveLimitSwitch;
    Encoder encoder = RobotMap.driveTrainEncoder;
    Gyro gyro = RobotMap.driveTrainGyro;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    
    private final double SMOOTH = 0.7;
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
        setDefaultCommand(new DriveWithJoystick());
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
	
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void processJoystick(Joystick stick){
    	robotDrive.tankDrive(SMOOTH * -stick.getAxis(Joystick.AxisType.kY),
    			SMOOTH * -stick.getRawAxis(5), true);
    }
    
    public void stop(){
    	robotDrive.tankDrive(0, 0);
    }
}

