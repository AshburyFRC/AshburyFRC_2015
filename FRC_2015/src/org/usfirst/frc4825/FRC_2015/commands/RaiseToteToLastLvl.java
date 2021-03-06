package org.usfirst.frc4825.FRC_2015.commands;

import org.usfirst.frc4825.FRC_2015.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RaiseToteToLastLvl extends Command {

	public RaiseToteToLastLvl() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.toteLift);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		System.out.println("Initialize RaiseToteToLastLvl");
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.toteLift.raiseTote();
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return !Robot.toteLift.getUpperSwitch();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		System.out.println("End RaiseToteToLastLvl");
		Robot.toteLift.stopTote();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
