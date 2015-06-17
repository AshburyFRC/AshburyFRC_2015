package org.usfirst.frc4825.FRC_2015.subsystems;

import java.util.Comparator;
import java.util.Vector;

import org.usfirst.frc4825.FRC_2015.Robot;
import org.usfirst.frc4825.FRC_2015.RobotMap;
import org.usfirst.frc4825.FRC_2015.commands.DriveWithJoystick;

import com.ni.vision.NIVision;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Ultra Pro Code By Isidor Ehrlich
 */
public class DriveTrain extends Subsystem {
	SpeedController leftSpeedController = RobotMap.driveTrainLeftSpeedController;
	SpeedController rightSpeedController = RobotMap.driveTrainRightSpeedController;
	RobotDrive robotDrive21 = RobotMap.driveTrainRobotDrive21;
	DigitalInput driveSwitch = RobotMap.driveTrainDriveSwitch;
	// Encoder encoder = RobotMap.driveTrainEncoder;
	Gyro gyro = RobotMap.driveTrainGyro;
	double angle = gyro.getAngle();
	int session = 0;

	private int reversed = 1;

	private double memY = 0;
	private double memX = 0;
	
	private double err = 0;
	private double pErr = 0;
	private double res = 0;
	
	private double time = 0;

	public void reset(){
		memY = 0;
		memX = 0;
		
		err = 0;
		pErr = 0;
		res = 0;
		time = 0;
	}
	
	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new DriveWithJoystick());
	}

	public void driveStraitNoSensor(double speed) {
		// Drive in a strait line at a given speed; negative is reverse
		robotDrive21.drive(speed, 0.0);
	}

	public void driveStraitSensor(double speed) {
		// Drive in a strait line at a given speed; negative is reverse; uses
		// gyro
		double angle = RobotMap.driveTrainGyro.getAngle(); // get current
															// heading
		robotDrive21.drive(speed, -angle * 0.03); // drive towards heading 0
		Timer.delay(0.004);
	}

	public void stop() {
		// Stop the drive train
		robotDrive21.drive(0.0, 0.0);
	}

	public void processJoystickInput(Joystick stick1) {
		// arcade drive
		System.out.println("Raw Axis (4)=>" + stick1.getRawAxis(4));
		int signX = (stick1.getRawAxis(4) > 0) ? 1 : -1;
		double speed = Math.pow(stick1.getAxis(Joystick.AxisType.kY), 3);
		double rotation = (Math.pow(stick1.getRawAxis(4), 5) * signX);
		robotDrive21.arcadeDrive(reversed * speed * 0.7, rotation * 0.8, true);
	}

	public void processJoystickInputExperimental(Joystick stick1) {
		double speed, rotation;
		final double MAX = 1.0;
		final double MIN = -1.0;

		speed = ((0.2) * memY + (0.8) * (stick1.getAxis(Joystick.AxisType.kY)));
		memY = speed;

		rotation = ((0.2) * memX + (0.8) * (-stick1.getRawAxis(4)));
		memX = rotation;
		if (rotation > MAX)
			rotation = MAX;
		if (rotation < MIN)
			rotation = MIN;

		robotDrive21.arcadeDrive( (stick1.getRawAxis(2)*0.5 + 1)*(reversed * speed * 0.5), rotation * 0.5, false);

		System.out.println(stick1.getAxis(Joystick.AxisType.kY));
		System.out.println(speed);
		System.out.println(stick1.getRawAxis(4));
		System.out.println(rotation);
	}
	
	public void processJoysticInputP(Joystick stick1){
		time++;
		
		double K = 0.5;
		
		double speed, yStick = stick1.getAxis(Joystick.AxisType.kY);
		double rotation = -stick1.getRawAxis(4);
		
		err = yStick - RobotMap.driveTrainLeftSpeedController.get();
		
		speed = K*err;
		System.out.println("PI: " + time + "; " + RobotMap.driveTrainLeftSpeedController.get() + " >" + yStick);
		robotDrive21.arcadeDrive(speed, rotation, false);
	}
	
	public void processJoystickInputPI(Joystick stick1){
		time++;
		
		double K = 0.5, tau_i = 0.3;
		
		double speed, yStick = stick1.getAxis(Joystick.AxisType.kY);
		double rotation = -stick1.getRawAxis(4);
		
		err = yStick - RobotMap.driveTrainLeftSpeedController.get();
		res += K/tau_i * err;
		
		speed = K*err + res;
		System.out.println("PI: " + time + "; " + RobotMap.driveTrainLeftSpeedController.get() + " >" + yStick);
		
		robotDrive21.arcadeDrive(speed, rotation, false);
	}
	
	public void processJoystickInputPID(Joystick stick1){
		time++;
		
		double K = 0.5, tau_i = 0.3, tau_d = 0.3;
		
		double speed, yStick = stick1.getAxis(Joystick.AxisType.kY);
		double rotation = -stick1.getRawAxis(4);
		
		err = yStick - RobotMap.driveTrainLeftSpeedController.get();
		res += K/tau_i * err;
		
		speed = K*err + res + ((pErr - err)*K/tau_d);
		pErr = err;
		
		System.out.println(time + "; " + RobotMap.driveTrainLeftSpeedController.get() + " >" + yStick);
		robotDrive21.arcadeDrive(speed, rotation, false);
	}
	
	public void processJoystickError(Joystick stick1){
		time++;
		
		double speed = stick1.getAxis(Joystick.AxisType.kY);
		double rotation = -stick1.getRawAxis(4);
		
		err = speed - RobotMap.driveTrainLeftSpeedController.get();
		System.out.println(speed);
		System.out.println(time + "; " + err);
		
		robotDrive21.arcadeDrive(speed, rotation);
	}

	public boolean atSwitch() {
		// Returns if limit switch is pressed
		return Robot.driveTrain.driveSwitch.get();
	}

	public boolean turn90DegreesLeft() {
		// Turns the robot to the right 90 degrees
		robotDrive21.tankDrive(-0.8, 0.8);
		if (gyro.getAngle() <= angle - 90) {
			return true;
		} else {
			return false;
		}
	}

	public void resetTurnAngle() {
		// Resets the saved starting angle and gyro
		gyro.reset();
		angle = gyro.getAngle();
	}

	public double getZAxis(Joystick stick) {
		return stick.getAxis(Joystick.AxisType.kZ);
	}

	public boolean getSwitch() {
		return driveSwitch.get();
	}

	public void toggleReversed() {
		if (reversed == 1)
			reversed = -1;
		else if (reversed == -1)
			reversed = 1;
	}

	public void turnWithSpeed(double speed) {
		robotDrive21.arcadeDrive(0.0, speed);
	}

	public double getGyroAngle() {
		return gyro.getAngle();
	}

	public void resetGyro() {
		gyro.reset();
	}

//	// \\Image Processing//\\
//
//	// Constants
//	NIVision.Range TOTE_HUE_RANGE = new NIVision.Range(24, 49); // Default hue
//																// range for
//																// yellow tote
//	NIVision.Range TOTE_SAT_RANGE = new NIVision.Range(67, 255); // Default
//																	// saturation
//																	// range for
//																	// yellow
//																	// tote
//	NIVision.Range TOTE_VAL_RANGE = new NIVision.Range(49, 255); // Default
//																	// value
//																	// range for
//																	// yellow
//																	// tote
//	double AREA_MINIMUM = 0.5; // Default Area minimum for particle as a
//								// percentage of total image area
//	double LONG_RATIO = 2.22; // Tote long side = 26.9 / Tote height = 12.1 =
//								// 2.22
//	double SHORT_RATIO = 1.4; // Tote short side = 16.9 / Tote height = 12.1 =
//								// 1.4
//	double SCORE_MIN = 75.0; // Minimum score to be considered a tote
//	double VIEW_ANGLE = 49.4; // View angle fo camera, set to Axis m1011 by
//								// default, 64 for m1013, 51.7 for 206, 52 for
//								// HD3000 square, 60 for HD3000 640x480
//	NIVision.ParticleFilterCriteria2 criteria[] = new NIVision.ParticleFilterCriteria2[1];
//	NIVision.ParticleFilterOptions2 filterOptions = new NIVision.ParticleFilterOptions2(
//			0, 0, 1, 1);
//	Scores scores = new Scores();
//	int toteAt = 0;
//
//	// A structure to hold measurements of a particle
//	public class ParticleReport implements Comparator<ParticleReport>,
//			Comparable<ParticleReport> {
//		double PercentAreaToImageArea;
//		double Area;
//		double ConvexHullArea;
//		double BoundingRectLeft;
//		double BoundingRectTop;
//		double BoundingRectRight;
//		double BoundingRectBottom;
//
//		@Override
//		public int compareTo(ParticleReport r) {
//			return (int) (r.Area - this.Area);
//		}
//
//		@Override
//		public int compare(ParticleReport r1, ParticleReport r2) {
//			return (int) (r1.Area - r2.Area);
//		}
//	};
//
//	// Structure to represent the scores for the various tests used for target
//	// identification
//	public class Scores {
//		double Trapezoid;
//		double LongAspect;
//		double ShortAspect;
//		double AreaToConvexHullArea;
//	};
//
//	// Uses vision processing to get to the bin
//	public void getToBin() {
//		// Will loop the image processing code until it the bin detected limit
//		// switch is pressed
//		while (!atSwitch()) {
//			// I think that this should grab the current image from the webcam
//			// on the front of the robot
//			NIVision.Image initialImage = null;
//			NIVision.Image processedImage = null;
//			RobotMap.frontCamera.getImage(initialImage);
//			// Threshold the image looking for yellow (tote color)
//			NIVision.imaqColorThreshold(initialImage, processedImage, 255,
//					NIVision.ColorMode.HSV, TOTE_HUE_RANGE, TOTE_SAT_RANGE,
//					TOTE_VAL_RANGE);
//			// Display thresholded image
//			CameraServer.getInstance().setImage(processedImage);
//			// Finding the largest particle (assume that it is the tote)
//			int connectivity8 = 1;// find what this does it should work with 1
//			int numParticles = NIVision.imaqCountParticles(processedImage,
//					connectivity8);
//			if (numParticles > 0) {
//				// Measure particles and sort by particle size
//				Vector<ParticleReport> particles = new Vector<ParticleReport>();
//				for (int particleIndex = 0; particleIndex < numParticles; particleIndex++) {
//					ParticleReport par = new ParticleReport();
//					par.PercentAreaToImageArea = NIVision.imaqMeasureParticle(
//							processedImage, particleIndex, 0,
//							NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA);
//					par.Area = NIVision.imaqMeasureParticle(processedImage,
//							particleIndex, 0, NIVision.MeasurementType.MT_AREA);
//					par.ConvexHullArea = NIVision.imaqMeasureParticle(
//							processedImage, particleIndex, 0,
//							NIVision.MeasurementType.MT_CONVEX_HULL_AREA);
//					par.BoundingRectTop = NIVision.imaqMeasureParticle(
//							processedImage, particleIndex, 0,
//							NIVision.MeasurementType.MT_BOUNDING_RECT_TOP);
//					par.BoundingRectLeft = NIVision.imaqMeasureParticle(
//							processedImage, particleIndex, 0,
//							NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT);
//					par.BoundingRectBottom = NIVision.imaqMeasureParticle(
//							processedImage, particleIndex, 0,
//							NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM);
//					par.BoundingRectRight = NIVision.imaqMeasureParticle(
//							processedImage, particleIndex, 0,
//							NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT);
//					particles.add(par);
//				}
//				particles.sort(null);
//				// Find the largest most bin-shaped particle
//				for (int particle = 0; particle < particles.size(); particle++) {
//					scores.Trapezoid = TrapezoidScore(particles
//							.elementAt(particle));
//					SmartDashboard.putNumber("Trapezoid", scores.Trapezoid);
//					scores.LongAspect = LongSideScore(particles
//							.elementAt(particle));
//					SmartDashboard.putNumber("Long Aspect", scores.LongAspect);
//					scores.ShortAspect = ShortSideScore(particles
//							.elementAt(particle));
//					SmartDashboard
//							.putNumber("Short Aspect", scores.ShortAspect);
//					scores.AreaToConvexHullArea = ConvexHullAreaScore(particles
//							.elementAt(particle));
//					SmartDashboard.putNumber("Convex Hull Area",
//							scores.AreaToConvexHullArea);
//					boolean isTote = scores.Trapezoid > SCORE_MIN
//							&& (scores.LongAspect > SCORE_MIN || scores.ShortAspect > SCORE_MIN)
//							&& scores.AreaToConvexHullArea > SCORE_MIN;
//					if (isTote) {
//						toteAt = particle;
//						break;
//					}
//				}
//				// Determine where the particle is in the image then turn and
//				// drive so that the particle is in the center
//
//			} else {
//				System.err.println("No Particles Found!");// No particles found
//															// error message
//				break;
//			}
//		}
//	}
//
//	//
//	// Provided Code
//	//
//	/**
//	 * Converts a ratio with ideal value of 1 to a score. The resulting function
//	 * is piecewise linear going from (0,0) to (1,100) to (2,0) and is 0 for all
//	 * inputs outside the range 0-2
//	 */
//	double ratioToScore(double ratio) {
//		return (Math.max(0, Math.min(100 * (1 - Math.abs(1 - ratio)), 100)));
//	}
//
//	/**
//	 * Method to score convex hull area. This scores how "complete" the particle
//	 * is. Particles with large holes will score worse than a filled in shape
//	 */
//	double ConvexHullAreaScore(ParticleReport report) {
//		return ratioToScore((report.Area / report.ConvexHullArea) * 1.18);
//	}
//
//	/**
//	 * Method to score if the particle appears to be a trapezoid. Compares the
//	 * convex hull (filled in) area to the area of the bounding box. The
//	 * expectation is that the convex hull area is about 95.4% of the bounding
//	 * box area for an ideal tote.
//	 */
//	double TrapezoidScore(ParticleReport report) {
//		return ratioToScore(report.ConvexHullArea
//				/ ((report.BoundingRectRight - report.BoundingRectLeft)
//						* (report.BoundingRectBottom - report.BoundingRectTop) * .954));
//	}
//
//	/**
//	 * Method to score if the aspect ratio of the particle appears to match the
//	 * long side of a tote.
//	 */
//	double LongSideScore(ParticleReport report) {
//		return ratioToScore(((report.BoundingRectRight - report.BoundingRectLeft) / (report.BoundingRectBottom - report.BoundingRectTop))
//				/ LONG_RATIO);
//	}
//
//	/**
//	 * Method to score if the aspect ratio of the particle appears to match the
//	 * short side of a tote.
//	 */
//	double ShortSideScore(ParticleReport report) {
//		return ratioToScore(((report.BoundingRectRight - report.BoundingRectLeft) / (report.BoundingRectBottom - report.BoundingRectTop))
//				/ SHORT_RATIO);
//	}
}
