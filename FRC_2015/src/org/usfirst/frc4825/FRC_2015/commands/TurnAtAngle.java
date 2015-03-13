package org.usfirst.frc4825.FRC_2015.commands;

import org.usfirst.frc4825.FRC_2015.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class TurnAtAngle extends Command {

	private double angle;
	private int sign = 1;

	public TurnAtAngle(double angle) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrain);

		this.angle = angle;
		if (angle < 0.0)
			sign = -1;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		System.out.println("initialize TurnAtAngle");
		Robot.driveTrain.resetGyro();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.driveTrain.turnWithSpeed(-sign
				* SmartDashboard.getNumber("Autonomus Speed") * 1.5); // / 2);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return ((Math.abs(angle - Robot.driveTrain.getGyroAngle())) < 0.1);
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		System.out.println("end TurnAtAngle");
		Robot.driveTrain.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
