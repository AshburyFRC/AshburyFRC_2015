package org.usfirst.frc4825.FRC_2015.subsystems;

import java.util.Comparator;
import java.util.Vector;

import org.usfirst.frc4825.FRC_2015.Robot;
import org.usfirst.frc4825.FRC_2015.RobotMap;
import org.usfirst.frc4825.FRC_2015.commands.*;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ImageType;

import edu.wpi.first.wpilibj.*;
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
	Encoder encoder = RobotMap.driveTrainEncoder;
	Gyro gyro = RobotMap.driveTrainGyro;
	double angle = gyro.getAngle();
	int session = 0;
	
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
		// Drive in a strait line at a given speed; negative is reverse; uses gyro
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
		int signX = (stick1.getRawAxis(4) > 1) ? 1 : -1;
		double speed = Math.pow(stick1.getAxis(Joystick.AxisType.kY), 3);
		double rotation = (Math.pow(stick1.getRawAxis(4), 5) * signX);
		if (rotation < 0.5 && rotation > -0.5) {// ensures the robot goes straight within the specified dead zone using a gyro
			Robot.driveTrain.driveStraitSensor(speed);
		} else {
			robotDrive21.arcadeDrive(speed, rotation);
		}
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
	
	
	
	//\\ Image Processing //\\
	
	
	
	//Constants
	NIVision.Range TOTE_HUE_RANGE = new NIVision.Range(24, 49);		//Default hue range for yellow tote
	NIVision.Range TOTE_SAT_RANGE = new NIVision.Range(67, 255);	//Default saturation range for yellow tote
	NIVision.Range TOTE_VAL_RANGE = new NIVision.Range(49, 255);	//Default value range for yellow tote
	double AREA_MINIMUM = 0.5;	//Default Area minimum for particle as a percentage of total image area
	double LONG_RATIO = 2.22; 	//Tote long side = 26.9 / Tote height = 12.1 = 2.22
	double SHORT_RATIO = 1.4; 	//Tote short side = 16.9 / Tote height = 12.1 = 1.4
	double SCORE_MIN = 75.0;  	//Minimum score to be considered a tote
	double VIEW_ANGLE = 49.4; 	//View angle fo camera, set to Axis m1011 by default, 64 for m1013, 51.7 for 206, 52 for HD3000 square, 60 for HD3000 640x480
	NIVision.ParticleFilterCriteria2 criteria[] = new NIVision.ParticleFilterCriteria2[1];
	NIVision.ParticleFilterOptions2 filterOptions = new NIVision.ParticleFilterOptions2(0,0,1,1);
	Scores scores = new Scores();
	
	//A structure to hold measurements of a particle
	public class ParticleReport implements Comparator<ParticleReport>, Comparable<ParticleReport>{
		double PercentAreaToImageArea;
		double Area;
		double ConvexHullArea;
		double BoundingRectLeft;
		double BoundingRectTop;
		double BoundingRectRight;
		double BoundingRectBottom;
		
		public int compareTo(ParticleReport r)
		{
			return (int)(r.Area - this.Area);
		}
		
		public int compare(ParticleReport r1, ParticleReport r2)
		{
			return (int)(r1.Area - r2.Area);
		}
	}
	
	//Structure to represent the scores for the various tests used for target identification
	public class Scores {
		double Trapezoid;
		double LongAspect;
		double ShortAspect;
		double AreaToConvexHullArea;
	}
	
	//Processing the Image Somehow
	public void processImage(){
		//The camera name (ex "cam0") can be found through the roborio web interface
	    session = NIVision.IMAQdxOpenCamera("cam0", NIVision.IMAQdxCameraControlMode.CameraControlModeController);
	    NIVision.IMAQdxConfigureGrab(session);
	    NIVision.IMAQdxStartAcquisition(session);
	    
		//Setting images to use
		Image image = NIVision.imaqCreateImage(ImageType.IMAGE_RGB, 0);
		Image binaryFrame = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 0);
		NIVision.IMAQdxGrab(session, image, 1);
		CameraServer.getInstance().setImage(image);
		
		//Threshold the image looking for yellow (tote color)
		NIVision.imaqColorThreshold(binaryFrame, image, 255, NIVision.ColorMode.HSV, TOTE_HUE_RANGE, TOTE_SAT_RANGE, TOTE_VAL_RANGE);
		
		//Count Particles
		int numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
		
		//Send masked image to dashboard to assist in tweaking mask.
		CameraServer.getInstance().setImage(binaryFrame);
		
		//Measure particles and sort by particle size
		Vector<ParticleReport> particles = new Vector<ParticleReport>();
		for(int particleIndex = 0; particleIndex < numParticles; particleIndex++)
		{
			ParticleReport par = new ParticleReport();
			par.PercentAreaToImageArea = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA);
			par.Area = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA);
			par.ConvexHullArea = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_CONVEX_HULL_AREA);
			par.BoundingRectTop = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_TOP);
			par.BoundingRectLeft = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT);
			par.BoundingRectBottom = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM);
			par.BoundingRectRight = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT);
			particles.add(par);
		}
		particles.sort(null);
		
		//This example only scores the largest particle. Extending to score all particles and choosing the desired one is left as an exercise for the reader.
		//Note that the long and short side scores expect a single tote and will not work for a stack of 2 or more totes.
		scores.Trapezoid = TrapezoidScore(particles.elementAt(0));
		scores.LongAspect = LongSideScore(particles.elementAt(0));
		scores.ShortAspect = ShortSideScore(particles.elementAt(0));
		scores.AreaToConvexHullArea = ConvexHullAreaScore(particles.elementAt(0));	
		//Detects data about the tote
		boolean isTote = scores.Trapezoid > SCORE_MIN && (scores.LongAspect > SCORE_MIN || scores.ShortAspect > SCORE_MIN) && scores.AreaToConvexHullArea > SCORE_MIN;
		boolean isLong = scores.LongAspect > scores.ShortAspect;
		double distance = computeDistance(binaryFrame, particles.elementAt(0), isLong);
		
		//Send distance and tote status to dashboard. The bounding rect, particularly the horizontal center (left - right) may be useful for rotating/driving towards a tote
		SmartDashboard.putBoolean("IsTote", isTote);
		SmartDashboard.putNumber("Distance", distance);
	}
	
	//Comparator function for sorting particles. Returns true if particle 1 is larger
	static boolean CompareParticleSizes(ParticleReport particle1, ParticleReport particle2)
	{
		//we want descending sort order
		return particle1.PercentAreaToImageArea > particle2.PercentAreaToImageArea;
	}

	/**
	 * Converts a ratio with ideal value of 1 to a score. The resulting function is piecewise
	 * linear going from (0,0) to (1,100) to (2,0) and is 0 for all inputs outside the range 0-2
	 */
	double ratioToScore(double ratio)
	{
		return (Math.max(0, Math.min(100*(1-Math.abs(1-ratio)), 100)));
	}

	/**
	 * Method to score convex hull area. This scores how "complete" the particle is. Particles with large holes will score worse than a filled in shape
	 */
	double ConvexHullAreaScore(ParticleReport report)
	{
		return ratioToScore((report.Area/report.ConvexHullArea)*1.18);
	}

	/**
	 * Method to score if the particle appears to be a trapezoid. Compares the convex hull (filled in) area to the area of the bounding box.
	 * The expectation is that the convex hull area is about 95.4% of the bounding box area for an ideal tote.
	 */
	double TrapezoidScore(ParticleReport report)
	{
		return ratioToScore(report.ConvexHullArea/((report.BoundingRectRight-report.BoundingRectLeft)*(report.BoundingRectBottom-report.BoundingRectTop)*.954));
	}

	/**
	 * Method to score if the aspect ratio of the particle appears to match the long side of a tote.
	 */
	double LongSideScore(ParticleReport report)
	{
		return ratioToScore(((report.BoundingRectRight-report.BoundingRectLeft)/(report.BoundingRectBottom-report.BoundingRectTop))/LONG_RATIO);
	}

	/**
	 * Method to score if the aspect ratio of the particle appears to match the short side of a tote.
	 */
	double ShortSideScore(ParticleReport report){
		return ratioToScore(((report.BoundingRectRight-report.BoundingRectLeft)/(report.BoundingRectBottom-report.BoundingRectTop))/SHORT_RATIO);
	}

	/**
	 * Computes the estimated distance to a target using the width of the particle in the image. For more information and graphics
	 * showing the math behind this approach see the Vision Processing section of the ScreenStepsLive documentation.
	 *
	 * @param image The image to use for measuring the particle estimated rectangle
	 * @param report The Particle Analysis Report for the particle
	 * @param isLong Boolean indicating if the target is believed to be the long side of a tote
	 * @return The estimated distance to the target in feet.
	 */
	double computeDistance (Image image, ParticleReport report, boolean isLong) {
		double normalizedWidth, targetWidth;
		NIVision.GetImageSizeResult size;

		size = NIVision.imaqGetImageSize(image);
		normalizedWidth = 2*(report.BoundingRectRight - report.BoundingRectLeft)/size.width;
		targetWidth = isLong ? 26.0 : 16.9;

		return  targetWidth/(normalizedWidth*12*Math.tan(VIEW_ANGLE*Math.PI/(180*2)));
	}
}
