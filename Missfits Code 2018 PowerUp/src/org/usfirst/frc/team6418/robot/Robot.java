//Robot-Code-2018 from Missfits github Acc
package org.usfirst.frc.team6418.robot;

import org.usfirst.frc.team6418.robot.commands.ExampleCommand;
import org.usfirst.frc.team6418.robot.subsystems.ExampleSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import java.util.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.buttons.*;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	//these next few lines of code create the mecanum drive interface for our bot 		

	// Channels for the wheels
	
	final DigitalInput intakeLeftLimit = new DigitalInput(3);
	final DigitalInput intakeRightLimit = new DigitalInput(2);
	final DigitalInput elevatorGroundLimit =new DigitalInput(0);
	final DigitalInput elevatorMaxLimit = new DigitalInput(1);
		
	final TalonSRX kFrontLeftChannel = new TalonSRX (2);
	final TalonSRX kRearLeftChannel = new TalonSRX (3);
	final TalonSRX kFrontRightChannel = new TalonSRX (1);
	final TalonSRX kRearRightChannel = new TalonSRX (4);
	
	VictorSP intakeRight = new VictorSP (0);
	VictorSP intakeLeft = new VictorSP (1);
	
	VictorSP climber1 = new VictorSP (2);
	VictorSP climber2 = new VictorSP (3);
	
	Spark elevatorMotor = new Spark(4);
	
	// The channel on the driver station that the joystick is connected to
	final int rightJoystickChannel = 0;
	final int leftJoystickChannel = 1;
	final int xBoxChannel = 2;

	Joystick rightStick = new Joystick(rightJoystickChannel);
	Joystick leftStick = new Joystick(leftJoystickChannel);
	Joystick xBox = new Joystick(xBoxChannel);

	//this next line will probably need to be changed... 
	public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	public static OI oi;

	public Compressor c = new Compressor (0);
	public DoubleSolenoid intakeSolenoid= new DoubleSolenoid(2, 3);
	public DoubleSolenoid climberSolenoid = new DoubleSolenoid(0,1);
	
	public static Timer myTimer = new Timer();
	public static Timer solenoidTimer = new Timer();
	
	public static ADXRS450_Gyro gyro = new ADXRS450_Gyro();

	public int xBoxA = 1;
	public int xBoxB = 2;
	public int xBoxX = 3;
	public int xBoxY = 4;
	public int xBoxLeftBumper = 5;
	public int xBoxRightBumper = 6;
	public int xBoxBack = 7;
	public int xBoxStart = 8;

	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();

	public int elevatorZone = 1;
	
	
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
		
		// copied from the Mecanum Drive example class; was in a separate class before: public Robot (){
	
		kFrontLeftChannel.setInverted(true);
		kRearLeftChannel.setInverted(true);
		// invert the left side motors
		// may need to change or remove to match the robot
		
		
		//operating compressor
		c.setClosedLoopControl(true);
	
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
		
		// solenoid.set(DoubleSolenoid.Value.kOff);
		// solenoid.set(DoubleSolenoid.Value.kForward);
		solenoidTimer.reset();
		solenoidTimer.start();

		gyro.reset();
		
		climberSolenoid.set(DoubleSolenoid.Value.kForward);
		
	}

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
		
		boolean elevatorGroundLimitPressed = !elevatorGroundLimit.get();
		boolean elevatorMaxLimitPressed = !elevatorMaxLimit.get();
		boolean intakeLeftLimitPressed = intakeLeftLimit.get();
		boolean intakeRightLimitPressed = !intakeRightLimit.get();
		//intake left is wired normally open instead of closed
		
		double xBoxLeftTrigger = xBox.getRawAxis(2);
		double xBoxRightTrigger = xBox.getRawAxis(3);
		double xBoxLeftJoystickY = xBox.getRawAxis(1);
		double xBoxRightJoystickY = xBox.getRawAxis(5);
		
		double leftJoystickX = leftStick.getX();
		double leftJoystickY = leftStick.getY();
		double rightJoystickX = rightStick.getX();
		double rightJoystickY = rightStick.getY();
		 //gonna have to put boolean to make sure climber code doesn't run unless at right height
				
		if (xBox.getRawButton(xBoxStart)){
			climber1.set(0.5);
			climber2.set(0.5);
			//we don't want to use the joystick for the climber; use the START button to climb once it gets to X climber height
		} else {
			climber1.set(0);
			climber2.set(0);
		}
		
		
		if (Math.abs(xBoxRightJoystickY) > 0.1){
			if (xBoxRightJoystickY < 0 && !elevatorGroundLimitPressed) {
				elevatorMotor.set(xBoxRightJoystickY);
			}
			else if (xBoxRightJoystickY > 0 && !elevatorMaxLimitPressed) {
				elevatorMotor.set(xBoxRightJoystickY);
			}
			//elevator manual climbing
		}
		else {
			elevatorMotor.set(0);
		}
		/*Limit switches:
		pressed  = false
		not pressed = true	
		because that makes SENSE*/

		//controls intake wheels
		if(xBoxRightTrigger > 0){
			intakeRight.set(0.8);
			intakeLeft.set(0.8);
		}
		else if (xBoxLeftTrigger > 0){
			//if (intakeRightLimitPressed) {
				intakeRight.set(-0.8);
			//}else {
				//intakeRight.set(0);
			//}
			//if (!intakeLeftLimitPressed) {
				intakeLeft.set(-0.8);
			//}else {
				//intakeLeft.set(0);
			//}
		}
		else {
			intakeRight.set(0);
			intakeLeft.set(0);
		}
		
		
		//double minX = Math.min(Math.abs(leftJoystickX), Math.abs(rightJoystickX));
		if (Math.abs(rightJoystickX) > 0.2) {
			kFrontLeftChannel.set(ControlMode.PercentOutput,-rightJoystickX);
			kRearRightChannel.set(ControlMode.PercentOutput,-rightJoystickX);
			kFrontRightChannel.set(ControlMode.PercentOutput,rightJoystickX);
			kRearLeftChannel.set(ControlMode.PercentOutput,rightJoystickX);
			//manual strafing
		} else {
			kFrontLeftChannel.set(ControlMode.PercentOutput,leftJoystickY);
			kRearLeftChannel.set(ControlMode.PercentOutput,leftJoystickY);
			kFrontRightChannel.set(ControlMode.PercentOutput,rightJoystickY);
			kRearRightChannel.set(ControlMode.PercentOutput,rightJoystickY);
			//manual tank drive
		}
		
		/* get the decoded pulse width encoder position, 4096 units per rotation */
		int leftPulseWidthPos = kRearLeftChannel.getSensorCollection().getPulseWidthPosition(); 
		int rightPulseWidthPos = kRearRightChannel.getSensorCollection().getPulseWidthPosition(); 
		/* get measured velocity in units per 100ms, 4096 units is one rotation */ 
		int leftPulseWidthVel = kRearLeftChannel.getSensorCollection().getPulseWidthVelocity();
		int rightPulseWidthVel = kRearRightChannel.getSensorCollection().getPulseWidthVelocity();
		
		SmartDashboard.putNumber("Left Encoder Position", leftPulseWidthPos);
		SmartDashboard.putNumber("Right Encoder Position", rightPulseWidthPos);
		SmartDashboard.putNumber("Left Encoder Velocity", leftPulseWidthVel);
		SmartDashboard.putNumber("Right Encoder Velocity", rightPulseWidthVel);
		//SmartDashboard.putBoolean("Left Intake Limit Switch Pressed", intakeLeftLimitPressed);
		//SmartDashboard.putBoolean("Right Intake Limit Switch Pressed", intakeRightLimitPressed);
		SmartDashboard.putBoolean("Elevator Ground Limit Pressed", elevatorGroundLimitPressed);
		SmartDashboard.putBoolean("Elevator Max Limit Pressed", elevatorMaxLimitPressed);
		
		
		//robotDrive.driveCartesian(rightJoystickY, -rightJoystickX, -rightStick.getZ(), 0);
		//robotDrive2.tankDrive(leftStick.getY(), rightStick.getY());

		//makes pneumatics shoot out if right trigger is pressed and shoot back in when released
		if(xBox.getRawButton(xBoxRightBumper)){
			//button 6 is right bumper
			intakeSolenoid.set(DoubleSolenoid.Value.kForward);
		}
		else if(xBox.getRawButton(xBoxLeftBumper)){
			intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
		}
		else{
			intakeSolenoid.set(DoubleSolenoid.Value.kOff);
		}
			
	/*	here: if the top limit switch has not been pressed yet
			you can keep moving up
		if it is moving downward && the bottom limit switch has not been pressed 
		should we have a state variable? 
		do all this logic in the elevator subsystem code. ;
	*/
		
		if (xBox.getRawButton(xBoxBack)) {
			//7 SHOULD BE THE BACK BUTTON
			climberSolenoid.set(DoubleSolenoid.Value.kReverse);
		}
		else if (xBox.getRawButton(xBoxStart)){
			//8 should be START button
			climberSolenoid.set(DoubleSolenoid.Value.kForward);
		}
		
		
		
		//we can change it to toggle with the button later.... talk to the drivers. 

		
		//right joystick for forwards/back and strafing
		//left joystick controlls yaw (spinning)g
		
		
		//pretty sure the gyro might be able to align the robot to the field, it will turn based on the field
		//IF YOU ADD ANOTHER PARAMETER, THE GYRO ANGLE, IT BECOMES FIELD-ORIENTED
		//WE SHOULD DO THIS
		//field oriented is much easier to drive
		//if joystick goes forward, the robot moves away from the driver regardless of the robot's orientation!
		
		//https://www.pdocs.kauailabs.com/navx-mxp/examples/field-oriented-drive/
//		robotDrive.mecanumDrive_Cartesian(stick.getX(), stick.getY(), stick.getTwist(), gyro.getAngle());
		
		//we have to pick one or the other for the two lines^^
		
		//trying to operate pneumatics 2/2/18
		
	    // solenoid.set(DoubleSolenoid.Value.kReverse);
	

		
    }
	
	//lets see if i can push changes :)

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
	
	/*public int MAKINGTHISAREDXcheckZone(boolean groundSwitchPressed, boolean switchSwitchPressed, boolean scaleSwitchPressed, 
			boolean maxSwitchPressed, int currentZone, double elevatorSpeed) {
		int zone;
		if (groundSwitchPressed)
			zone = 1;
		else if (currentZone == 1 && !groundSwitchPressed && elevatorSpeed > 0)
			zone = 2;
		
		return zone;
	}*/
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
}
