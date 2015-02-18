package org.usfirst.frc4825.FRC_2015.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc4825.FRC_2015.Robot;

/**
 *
 */
public class TurnAtAngle extends Command {
	
	private double angle;

    public TurnAtAngle(double angle) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
    	
    	this.angle = angle;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("initialize TurnAtAngle");
    	Robot.driveTrain.resetGyro();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveTrain.turnWithSpeed(0.5);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return ((Robot.driveTrain.getGyroAngle() - angle) < 1);
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveTrain.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
