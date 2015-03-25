// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package org.usfirst.frc4825.FRC_2015.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Ultra Pro Code By Isidor Ehrlich &&& >>> $WAG $LAVA <<<
 */
public class AutonomousCommand extends CommandGroup {

	public AutonomousCommand() {
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

		//if (SmartDashboard.getNumber("Autonomus for Bin0; Tote1") == 1) {
			/*
			addSequential(new LowerToteToFirstLvl());
			addSequential(new RaiseToteToMiddleLvl());
			addSequential(new DriveWithTimer(-2.8));
			addSequential(new LowerToteToFirstLvl());
			addSequential(new DriveWithTimer(-0.5));
			*/
		//} else if (SmartDashboard.getNumber("Autonomus for Bin0; Tote1") == 0) {
			addSequential(new RaiseBinMiddleLvl());
			//addSequential(new RaiseBinToLastLvl());
			addSequential(new Delay(0.2));
			addSequential(new DriveWithTimer(2.9));
			addSequential(new Delay(0.2));
			addParallel(new LowerBinToFirstLvl()); //GOLDEN NUMBERS 3.0 and 0.6!!!1!11!one
			addSequential(new Delay(0.2));
			addSequential(new DriveWithTimer(0.7));
			
			addSequential(new Delay(0.2));
			addSequential(new TurnAtAngle(-80));
		//}

	}
}
