package org.usfirst.frc4825.FRC_2015.commands;

import org.usfirst.frc4825.FRC_2015.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Ultra Pro Code By Isidor Ehrlich
 */
public class DriveWithTimer extends Command {

	private double time;
	private boolean isForward = true;

	// Constructor
	public DriveWithTimer(double timeOfDriving) {
		requires(Robot.driveTrain);

		if (timeOfDriving < 0) {
			isForward = false;
			time = timeOfDriving * (-1);
		} else
			time = timeOfDriving;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		System.out.println("initialize timeOfDriving()");
		setTimeout(time);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if (!isForward)
			Robot.driveTrain.driveStraitNoSensor(SmartDashboard
					.getNumber("Autonomus Speed"));
		else
			Robot.driveTrain.driveStraitNoSensor(-SmartDashboard
					.getNumber("Autonomus Speed"));
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isTimedOut();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.driveTrain.stop();
		System.out.println("end DriveWithTimer()");
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
