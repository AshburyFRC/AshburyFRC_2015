package org.usfirst.frc4825.FRC_2015.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class GoToNearTote extends CommandGroup {

	public GoToNearTote() {
		// Add Commands here:
		// e.g. addSequential(new Command1());
		// addSequential(new Command2());
		// these will run in order.

		// To run multiple commands at the same time,
		// use addParallel()
		// e.g. addParallel(new Command1());
		// addSequential(new Command2());
		// Command1 and Command2 will run in parallel.

		// A command group will require all of the subsystems that each member
		// would require.
		// e.g. if Command1 requires chassis, and Command2 requires arm,
		// a CommandGroup containing them would require both the chassis and the
		// arm.

		addSequential(new DriveWithTimer(-1.0));
		addSequential(new TurnAtAngle(-85));
		addSequential(new DriveWithTimer(-2.0));
		addSequential(new TurnAtAngle(85));
	}
}
