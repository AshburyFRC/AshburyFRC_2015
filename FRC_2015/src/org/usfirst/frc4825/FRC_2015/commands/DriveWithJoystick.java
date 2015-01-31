package org.usfirst.frc4825.FRC_2015.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc4825.FRC_2015.Robot;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc4825.FRC_2015.Robot;
import org.usfirst.frc4825.FRC_2015.RobotMap;

/**
 *Ultra Pro Code By Isidor Ehrlich
 */
public class  DriveWithJoystick extends Command {

    public DriveWithJoystick() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);

        requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("Initilize DriveWithJoysticks");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveTrain.processJoystickInput(Robot.oi.getJoystick());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
