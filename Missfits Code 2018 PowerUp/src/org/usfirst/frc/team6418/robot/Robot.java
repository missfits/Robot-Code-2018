//Robot-Code-2018 from Missfits github Acc
package org.usfirst.frc.team6418.robot;

import org.usfirst.frc.team6418.robot.commands.ExampleCommand;
import org.usfirst.frc.team6418.robot.subsystems.ExampleSubsystem;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
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
	MecanumDrive robotDrive;
	DifferentialDrive robotDrive2;

	// Channels for the wheels
	
	final DigitalInput limitSwitch1 = new DigitalInput(0);
		
	final WPITalon kFrontLeftChannel = new WPITalon (2);
	final WPITalon kRearLeftChannel = new WPITalon (3);
	final WPITalon kFrontRightChannel = new WPITalon (1);
	final WPITalon kRearRightChannel = new WPITalon (4);
	
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

	// DoubleSolenoid solenoid = new DoubleSolenoid(1, 2);
	public Compressor c = new Compressor (0);
	public DoubleSolenoid intakeSolenoid= new DoubleSolenoid(2, 3);
	public static Timer myTimer = new Timer();
	public static Timer solenoidTimer = new Timer();
	
	public static ADXRS450_Gyro gyro = new ADXRS450_Gyro();

	
	
	public Button trigger = new JoystickButton(rightStick,1);
	public Button xBoxA = new JoystickButton(xBox,1);
	public Button xBoxB = new JoystickButton(xBox,2);
	public Button xBoxX = new JoystickButton(xBox,3);
	public Button xBoxY = new JoystickButton(xBox,4);
	public Button xBoxLeftBumper = new JoystickButton(xBox,5);
	public Button xBoxRightBumper = new JoystickButton(xBox,6);
	public Button xBoxStart = new JoystickButton(xBox,8);
	
	
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
		
		// copied from the Mecanum Drive example class; was in a separate class before: public Robot (){
		robotDrive = new MecanumDrive(kFrontLeftChannel, kRearLeftChannel, kFrontRightChannel, kRearRightChannel);
	
		kFrontLeftChannel.setInverted(true);
		kRearLeftChannel.setInverted(true);
		// invert the left side motors
		// may need to change or remove to match the robot
		
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
		
		// solenoid.set(DoubleSolenoid.Value.kOff);
		// solenoid.set(DoubleSolenoid.Value.kForward);
		solenoidTimer.reset();
		solenoidTimer.start();

		gyro.reset();
		
		
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
		
		double xBoxLeftTrigger = xBox.getRawAxis(2);
		double xBoxRightTrigger = xBox.getRawAxis(3);
		double xBoxLeftJoystickY = xBox.getRawAxis(1);
		double xBoxRightJoystickY = xBox.getRawAxis(5);
		
		double leftJoystickX = leftStick.getX();
		double leftJoystickY = leftStick.getY();
		double rightJoystickX = rightStick.getX();
		double rightJoystickY = rightStick.getY();
		 //gonna have to put boolean to make sure climber code doesn't run unless at right height
		
		
		
		if (Math.abs(xBoxLeftJoystickY) > 0.1){
			climber1.set(xBoxLeftJoystickY);
			climber2.set(xBoxLeftJoystickY);
			//we don't want to use the joystick for the climber; use the START button to climb once it gets to X climber height
		}
		
		
		if (Math.abs(xBoxRightJoystickY) > 0.1){
			elevatorMotor.set(xBoxRightJoystickY);
			//elevator manual climbing
		}
		

		if(xBoxRightTrigger > 0){
			intakeRight.set(0.8);
			intakeLeft.set(0.8);
		}
		else if (xBoxLeftTrigger > 0){
			intakeRight.set(-0.8);
			intakeLeft.set(-0.8);
		}
		else {
			intakeRight.set(0);
			intakeLeft.set(0);
		}
		
		
		//double minX = Math.min(Math.abs(leftJoystickX), Math.abs(rightJoystickX));
		if (Math.abs(rightJoystickX) > 0.2) {
			kFrontLeftChannel.set(-rightJoystickX);
			kRearRightChannel.set(-rightJoystickX);
			kFrontRightChannel.set(rightJoystickX);
			kRearLeftChannel.set(rightJoystickX);
			//manual strafing
		} else {
			kFrontLeftChannel.set(leftJoystickY);
			kRearLeftChannel.set(leftJoystickY);
			kFrontRightChannel.set(rightJoystickY);
			kRearRightChannel.set(rightJoystickY);
			//manual tank drive
		}
		//robotDrive.driveCartesian(rightJoystickY, -rightJoystickX, -rightStick.getZ(), 0);
		//robotDrive2.tankDrive(leftStick.getY(), rightStick.getY());
		/*if(Math.abs(rightJoystickX) > 0.1 || Math.abs(leftJoystickX)> 0.1){
			robotDrive.driveCartesian(rightStick.getX(), rightStick.getY(), leftStick.getX(), gyro.getAngle());
		}else{
			//robotDrive2.tankDrive(leftStick.getY(), rightStick.getY());
		}*/
		//makes pneumatics shoot out if right trigger is pressed and shoot back in when released
		if(xBox.getRawButton(6)){
			//button 6 is right bumper
			intakeSolenoid.set(DoubleSolenoid.Value.kForward);
		}
		else if(xBox.getRawButton(5)){
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
}
