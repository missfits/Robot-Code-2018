//Robot-Code-2018 from Missfits github Acc
package org.usfirst.frc.team6418.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team6418.robot.commands.ExampleCommand;
import org.usfirst.frc.team6418.robot.subsystems.ExampleSubsystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
//import edu.wpi.first.wpilibj.RobotDrive;
//import edu.wpi.first.wpilibj.RobotDrive.MotorType;
//import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	//these next few lines of code create the mecanum drive interface for our bot 		
//	RobotDrive robotDrive;
	MecanumDrive robotDrive;

	// Channels for the wheels
	//changed them from ints to Spark SpeedControllers to work with MecanumDrive
	final Spark kFrontLeftChannel = new Spark (2);
	final Spark kRearLeftChannel = new Spark (3);
	final Spark kFrontRightChannel = new Spark (1);
	final Spark kRearRightChannel = new Spark (0);
	

	// The channel on the driver station that the joystick is connected to
	final int kJoystickChannel = 0;

	Joystick stick = new Joystick(kJoystickChannel);

	//this next line will probably need to be changed... 
	public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	public static OI oi;

	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		oi = new OI();
		chooser.addDefault("Default Auto", new ExampleCommand());
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", chooser);
		
	//copied from the Mecanum Drive example class	
		//was in a separate class before, public Robot (){
		robotDrive = new MecanumDrive(kFrontLeftChannel, kRearLeftChannel, kFrontRightChannel, kRearRightChannel);
	
		kFrontLeftChannel.setInverted(true);
		kRearLeftChannel.setInverted(true);
		// invert the left side motors
		//may need to change or remove to match the robot
		
		robotDrive.setExpiration(0.1);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		autonomousCommand = chooser.getSelected();

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (autonomousCommand != null)
			autonomousCommand.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null)
			autonomousCommand.cancel();
	}
	
	
	AHRS ahrs;
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		//next line is from FRC Screen Steps Live 
		//https://wpilib.screenstepslive.com/s/currentCS/m/java/l/599704-driving-a-robot-using-mecanum-drive
		//use x and y to move forward or strafe (sliding), use z (twisty) axis to turn
		//driver has to have the "sitting on the robot" mindset/mentality - "robot-oriented" driving
//		robotDrive.mecanumDrive_Cartesian(stick.getX(), stick.getY(), stick.getTwist(),0);
		
		// robotDrive.driveCartesian(stick.getX(), stick.getY(), stick.getTwist(), 0.0);
		
		//pretty sure the gyro might be able to align the robot to the field, it will turn based on the field
		//IF YOU ADD ANOTHER PARAMETER, THE GYRO ANGLE, IT BECOMES FIELD-ORIENTED
		//WE SHOULD DO THIS
		//field oriented is much easier to drive
		//if joystick goes forward, the robot moves away from the driver regardless of the robot's orientation!
		
		//https://www.pdocs.kauailabs.com/navx-mxp/examples/field-oriented-drive/
//		robotDrive.mecanumDrive_Cartesian(stick.getX(), stick.getY(), stick.getTwist(), gyro.getAngle());
		
		//we have to pick one or the other for the two lines^^
		
		robotDrive.setSafetyEnabled(true);
	     	while (isOperatorControl() && isEnabled()) {
	     		if (stick.getRawButton(0)) {
	     			ahrs.reset();
	     		} try {
	     			/* Use the joystick X axis for lateral movement,
	     			Y axis for forward movement, and Z axis for rotation.
	     			Use navX-MXP yaw angle to define Field-centric transform */
	        	  	robotDrive.mecanumDrive_Cartesian(stick.getX(), stick.getY(), stick.getTwist(), ahrs.getAngle());
	          } catch (RuntimeException ex) {}
	          Timer.delay(0.005); // wait for a motor update time
	      }
    }
	
	//lets see if i can push changes :)

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}
