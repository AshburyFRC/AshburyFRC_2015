package org.usfirst.frc4825.FRC_2015.commands;

import org.usfirst.frc4825.FRC_2015.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Ultra Pro Code By Isidor Ehrlich
 */
public class DriveWithJoystick extends Command {

	public DriveWithJoystick() {
		requires(Robot.driveTrain);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		System.out.println("Initilize DriveWithJoysticks");
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// Robot.driveTrain.processJoystickInput(Robot.oi.getJoystick());
		//Robot.driveTrain.processJoystickInputExperimental(Robot.oi
				//.getJoystick());
		
		//Robot.driveTrain.processJoystickInputPI(Robot.oi
				//.getJoystick());
		
		//Robot.driveTrain.processJoystickInputPD(Robot.oi
			//.getJoystick());
		
		Robot.driveTrain.processJoystickInputPID(Robot.oi
			.getJoystick());
		
		//Robot.driveTrain.processJoystickError(Robot.oi.getJoystick());
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;// This should run until the code has it's execution force
						// terminated when the robot gets disabled at the end of
						// the match
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.driveTrain.reset();
		Robot.driveTrain.stop();
		System.out.println("End DriveWithJoysticks");
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
