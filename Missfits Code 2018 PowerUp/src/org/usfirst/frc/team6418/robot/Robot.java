//Robot-Code-2018 from Missfits github Acc
package org.usfirst.frc.team6418.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//TODO
//import com.kauailabs.navx.frc.AHRS;

enum StartingPosition {
	LEFT, MIDDLE, RIGHT
}

enum XBoxButtons {
	DONOTUSE, A, B, X, Y, LEFT_BUMPER, RIGHT_BUMPER, BACK, START
}

enum XBoxAxes {
	DONOTUSE, LEFT_Y, LEFT_TRIGGER, RIGHT_TRIGGER, DONOTUSE2, RIGHT_Y
}

public class Robot extends IterativeRobot {

	// AHRS ahrs = new AHRS(SerialPort.Port.kMXP); /* Alternatives: SPI.Port.kMXP,
	// I2C.Port.kMXP or SerialPort.Port.kUSB */

	final DigitalInput elevatorGroundLimit = new DigitalInput(0);
	final DigitalInput elevatorMaxLimit = new DigitalInput(1);

	final TalonSRX kFrontLeftChannel = new TalonSRX(2);
	final TalonSRX kRearLeftChannel = new TalonSRX(3);
	final TalonSRX kFrontRightChannel = new TalonSRX(1);
	final TalonSRX kRearRightChannel = new TalonSRX(4);

	// TODO
	// VictorSP intakeRight = new VictorSP(0);
	// VictorSP intakeLeft = new VictorSP(1);
	Spark intakeRight = new Spark(0);
	Spark intakeLeft = new Spark(1);

	VictorSP climber1 = new VictorSP(2);
	VictorSP climber2 = new VictorSP(3);

	Spark elevatorMotor = new Spark(4);

	// The channel on the driver station that the joystick is connected to
	final int rightJoystickChannel = 0;
	final int leftJoystickChannel = 1;
	final int xBoxChannel = 2;

	Joystick rightStick = new Joystick(rightJoystickChannel);
	Joystick leftStick = new Joystick(leftJoystickChannel);
	Joystick xBox = new Joystick(xBoxChannel);

	public Compressor compressor = new Compressor(0);
	public DoubleSolenoid intakeSolenoid = new DoubleSolenoid(2, 3);
	public DoubleSolenoid climberSolenoid = new DoubleSolenoid(0, 1);

	public static Timer autoTimer = new Timer();
	public static Timer solenoidTimer = new Timer();

	public int autoState = 0;

	public int switchIsLeftState;

	public static ADXRS450_Gyro gyro = new ADXRS450_Gyro();

	SendableChooser<StartingPosition> startPosition = new SendableChooser<>();
	SendableChooser<Boolean> usingEncoders = new SendableChooser<>();

	public int elevatorZone = 1;
	
	public double leftEncoderOffset;
	public double rightEncoderOffset;
	

	@Override
	public void robotInit() {
		startPosition.addDefault("Left", StartingPosition.LEFT);
		startPosition.addObject("Middle", StartingPosition.MIDDLE);
		startPosition.addObject("Right", StartingPosition.RIGHT);
		SmartDashboard.putData("Starting Position", startPosition);

		usingEncoders.addDefault("Encoders", true);
		usingEncoders.addObject("Timer", false);
		SmartDashboard.putData("Using Encoders", usingEncoders);

		kFrontLeftChannel.setInverted(true);
		kRearLeftChannel.setInverted(true);
		// TODO may need to change or remove to match the robot

		// operating compressor
		compressor.setClosedLoopControl(true);

		closeIntake();

	}

	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {

		// Scheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		autoState = 0;
		switchIsLeftState = switchIsLeft();
		leftEncoderOffset = kRearLeftChannel.getSensorCollection().getPulseWidthPosition();
		rightEncoderOffset = kRearRightChannel.getSensorCollection().getPulseWidthPosition();
		gyro.reset();
	}

	@Override
	public void autonomousPeriodic() {
		// Scheduler.getInstance().run();
		if ((switchIsLeftState == 1 && startPosition.getSelected() == StartingPosition.LEFT)
				|| (switchIsLeftState == 0 && startPosition.getSelected() == StartingPosition.RIGHT)) {
			driveStraightSwitch();
		} else if (startPosition.getSelected() == StartingPosition.MIDDLE) {
			middleAuto();
		} else {
			driveStraightOnly();
		}
		SmartDashboard.putNumber("Auto State:", autoState);
		smartDashboardEncoders();
	}

	@Override
	public void teleopInit() {

		// gyro.reset();

		climberSolenoid.set(DoubleSolenoid.Value.kForward);

	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();

		boolean elevatorGroundLimitPressed = elevatorGroundLimit.get();
		boolean elevatorMaxLimitPressed = elevatorMaxLimit.get();
		// intake left is wired normally open instead of closed

		double leftJoystickY = leftStick.getY();
		double rightJoystickX = rightStick.getX();
		double rightJoystickY = rightStick.getY();

		double elevatorJoystickY = -getAxis(XBoxAxes.RIGHT_Y);
		// gonna have to put boolean to make sure climber code doesn't run unless at
		// right height

		if (buttonIsPressed(XBoxButtons.START)) {
			climber1.set(0.5);
			climber2.set(0.5);
		} else {
			climber1.set(0);
			climber2.set(0);
		}

		// TODO
		if (elevatorJoystickY < -0.1 && !elevatorGroundLimitPressed) {
			elevatorMotor.set(elevatorJoystickY);
			SmartDashboard.putString("Driving the elevator", "DOWN");
		} else if (elevatorJoystickY > 0.1 && !elevatorMaxLimitPressed) {
			elevatorMotor.set(elevatorJoystickY);
			SmartDashboard.putString("Driving the elevator", "UP");
		} else {
			elevatorMotor.set(0);
			SmartDashboard.putString("Driving the elevator", "OFF");
		}

		// -----controls intake wheels-----
		// in
		if (getAxis(XBoxAxes.LEFT_TRIGGER) > 0.2) {
			intakeRight.set(-0.8);
			intakeLeft.set(-0.8);
		} else if (getAxis(XBoxAxes.RIGHT_TRIGGER) > 0.2) {
			intakeLeft.set(0.8);
			intakeRight.set(0.8);
		} else {
			intakeLeft.set(0);
			intakeRight.set(0);
		}

		// double minX = Math.min(Math.abs(leftJoystickX), Math.abs(rightJoystickX));
		if (Math.abs(rightJoystickX) > 0.2) {
			kFrontLeftChannel.set(ControlMode.PercentOutput, -rightJoystickX);
			kRearRightChannel.set(ControlMode.PercentOutput, -rightJoystickX);
			kFrontRightChannel.set(ControlMode.PercentOutput, rightJoystickX);
			kRearLeftChannel.set(ControlMode.PercentOutput, rightJoystickX);
			// manual strafing
		} else {
			kFrontLeftChannel.set(ControlMode.PercentOutput, leftJoystickY);
			kRearLeftChannel.set(ControlMode.PercentOutput, leftJoystickY);
			kFrontRightChannel.set(ControlMode.PercentOutput, rightJoystickY);
			kRearRightChannel.set(ControlMode.PercentOutput, rightJoystickY);
			// manual tank drive
		}

		// TODO
		SmartDashboard.putBoolean("Elevator Ground Limit Pressed", elevatorGroundLimitPressed);
		SmartDashboard.putBoolean("Elevator Max Limit Pressed", elevatorMaxLimitPressed);
		smartDashboardEncoders();
		// SmartDashboard.putNumber("NavX Angle", ahrs.getYaw());

		if (buttonIsPressed(XBoxButtons.RIGHT_BUMPER)) {
			intakeSolenoid.set(DoubleSolenoid.Value.kForward);
		} else if (buttonIsPressed(XBoxButtons.LEFT_BUMPER)) {
			intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
		} else {
			intakeSolenoid.set(DoubleSolenoid.Value.kOff);
		}

		/*
		 * here: if the top limit switch has not been pressed yet you can keep moving up
		 * if it is moving downward && the bottom limit switch has not been pressed
		 * should we have a state variable? do all this logic in the elevator subsystem
		 * code. ;
		 */

		if (buttonIsPressed(XBoxButtons.BACK)) {
			climberSolenoid.set(DoubleSolenoid.Value.kReverse);
		} else if (buttonIsPressed(XBoxButtons.START)) {
			climberSolenoid.set(DoubleSolenoid.Value.kForward);
		}
	}

	@Override
	public void testPeriodic() {
	}

	public boolean buttonIsPressed(XBoxButtons button) {
		return xBox.getRawButton(button.ordinal());
	}

	public double getAxis(XBoxAxes axis) {
		return xBox.getRawAxis(axis.ordinal());
	}

	public void driveStraightEncoder(int distance) {
		double encoderDistance = (distance / 18.85) * 4096;
		int pulseWidthPos = kRearLeftChannel.getSensorCollection().getPulseWidthPosition();
		if (pulseWidthPos < encoderDistance) {
			kFrontLeftChannel.set(ControlMode.PercentOutput, .5);
			kRearRightChannel.set(ControlMode.PercentOutput, .5);
			kFrontRightChannel.set(ControlMode.PercentOutput, .5);
			kRearLeftChannel.set(ControlMode.PercentOutput, .5);
		} else {
			stopDrive();
		}
	}

	public void driveStraight(double speed) {

		kFrontLeftChannel.set(ControlMode.PercentOutput, speed);
		kRearRightChannel.set(ControlMode.PercentOutput, speed);
		kFrontRightChannel.set(ControlMode.PercentOutput, speed);
		kRearLeftChannel.set(ControlMode.PercentOutput, speed);

	}

	public void runIntake(double speed) {
		intakeRight.set(speed);
		intakeLeft.set(speed);
	}

	public void openIntake() {
		intakeSolenoid.set(DoubleSolenoid.Value.kForward);
	}

	public void closeIntake() {
		intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
	}

	public void stopDrive() {
		kFrontLeftChannel.set(ControlMode.PercentOutput, 0.0);
		kRearRightChannel.set(ControlMode.PercentOutput, 0.0);
		kFrontRightChannel.set(ControlMode.PercentOutput, 0.0);
		kRearLeftChannel.set(ControlMode.PercentOutput, 0.0);
	}

	public void moveElevator(double speed) {
		// moving up is positive
		if (speed > 0 && !elevatorMaxLimit.get())
			elevatorMotor.set(speed);
		else if (speed < 0 && !elevatorGroundLimit.get())
			elevatorMotor.set(speed);
		elevatorMotor.set(0);
	}

	// battery voltage compensation:
	public double getVoltageCompensationMultipler() {
		return 12.8 / RobotController.getBatteryVoltage();
	}

	public int switchIsLeft() {
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		if (gameData.length() > 0) {
			if (gameData.charAt(0) == 'L') {
				return 1;
			} else {
				return 0;
			}
		}
		return -1;
	}

	public void driveStraightSwitch() {
		int pastState = autoState;
		switch (autoState) {
		case 0:
			autoTimer.reset();
			autoTimer.start();
			autoState++;
			break;
		case 1:
			if (checkIfNotDone(120, 2.0)) {
				// robot is 3'3", 38 in, 99 cm
				driveStraight(-0.5);
			} else {
				autoState++;
			}
			break;
		case 2:
			if (autoTimer.get() < 1.5)
				moveElevator(0.5);
			else {
				autoState++;
			}
			break;
		case 3:
			if (checkIfNotDone(15, 2.0)) {
				driveStraight(-0.25);
			} else {
				autoState++;
			}
			break;
		case 4:
			if (autoTimer.get() < 1.0)
				runIntake(0.8);
			else {
				openIntake();
				autoState++;
			}
			break;
		case 5:
			if (checkIfNotDone(20, 1.0)) {
				driveStraight(0.25);
			} else {
				autoState++;
			}
			break;

		default:
			stopDrive();
			runIntake(0);
			moveElevator(0);
			break;
		}
		if (pastState != autoState) {
			stopDrive();
			moveElevator(0);
			runIntake(0);
			leftEncoderOffset = kRearLeftChannel.getSensorCollection().getPulseWidthPosition();
			rightEncoderOffset = kRearRightChannel.getSensorCollection().getPulseWidthPosition();
			gyro.reset();
			autoTimer.reset();
			autoTimer.start();
		}
	}

	public void driveStraightOnly() {
		switch (autoState) {
		case 0:
			autoTimer.reset();
			autoTimer.start();
			leftEncoderOffset = kRearLeftChannel.getSensorCollection().getPulseWidthPosition();
			rightEncoderOffset = kRearRightChannel.getSensorCollection().getPulseWidthPosition();
			autoState++;
			break;
		case 1:
			if (checkIfNotDone(100, 1.5)) {
				driveStraight(-0.5);
			} else {
				stopDrive();
				autoTimer.reset();
				autoTimer.start();
				leftEncoderOffset = kRearLeftChannel.getSensorCollection().getPulseWidthPosition();
				rightEncoderOffset = kRearRightChannel.getSensorCollection().getPulseWidthPosition();
				autoState++;
			}
			break;
		/*
		 * case 2: //if(turnToAngle(90)) { stopDrive(); gyro.reset(); autoTimer.reset();
		 * autoTimer.start(); autoState++; // } break;
		 */
		default:
			stopDrive();
			runIntake(0);
			moveElevator(0);
			leftEncoderOffset = kRearLeftChannel.getSensorCollection().getPulseWidthPosition();
			rightEncoderOffset = kRearRightChannel.getSensorCollection().getPulseWidthPosition();
			break;
		}

	}

	public void middleAuto() {
		int pastState = autoState;
		double angle1 = 0, angle2 = 0;
		double turntDistance = 0;
		if (switchIsLeftState == -1)
			return;
		else if (switchIsLeftState == 1) {
			// is left TODO
			angle1 = -34.5;
			angle2 = -angle1;
			turntDistance = 108.04;
		} else if (switchIsLeftState == 0) {
			angle1 = 32.2;
			angle2 = -angle1;
			turntDistance = 105.15;
		}

		switch (autoState) {
		case 0:
			autoTimer.reset();
			autoTimer.start();
			autoState++;
			break;
		case 1:
			if (checkIfNotDone(12, 2.0)) {
				// robot is 3'3", 38 in, 99 cm
				driveStraight(-0.5);
			} else {
				autoState++;
			}
			break;
		case 2:
			if (checkIfNotTurnt(angle1))
				turnToAngle(angle1);
			else {
				autoState++;
			}
			break;
		case 3:
			if (checkIfNotDone(turntDistance, 2.0)) {
				driveStraight(-0.5);
				if (autoTimer.get() < 2) {
					moveElevator(0.5);
				} else {
					moveElevator(0);
				}
			} else {
				autoState++;
			}
			break;
		case 4:
			if (checkIfNotTurnt(angle2))
				turnToAngle(angle2);
			else {
				autoState++;
			}
			break;
		case 5:
			if (autoTimer.get() < 1.0)
				runIntake(0.8);
			else {
				openIntake();
				autoState++;
			}
			break;

		default:
			stopDrive();
			runIntake(0);
			moveElevator(0);
			break;
		}
		if (pastState != autoState) {
			stopDrive();
			moveElevator(0);
			runIntake(0);
			leftEncoderOffset = kRearLeftChannel.getSensorCollection().getPulseWidthPosition();
			rightEncoderOffset = kRearRightChannel.getSensorCollection().getPulseWidthPosition();
			gyro.reset();
			autoTimer.reset();
			autoTimer.start();
		}

	}

	/*
	 * public int switchIsLeft() { String gameData; gameData =
	 * DriverStation.getInstance().getGameSpecificMessage(); if (gameData.length() >
	 * 0) { if (gameData.charAt(0) == 'L') { return 1; } else { return 0; } } return
	 * -1; }
	 */

	public boolean checkIfNotTurnt(double angle) {
		return (Math.abs(gyro.getAngle()) < Math.abs(angle) * 0.9);
	}

	public void turnToAngle(double angle) {
		double speed = 0.3;
		if (angle < 0) {
			kFrontLeftChannel.set(ControlMode.PercentOutput, speed);
			kRearLeftChannel.set(ControlMode.PercentOutput, speed);
			kRearRightChannel.set(ControlMode.PercentOutput, -speed);
			kFrontRightChannel.set(ControlMode.PercentOutput, -speed);
		} else {
			kFrontLeftChannel.set(ControlMode.PercentOutput, -speed);
			kRearLeftChannel.set(ControlMode.PercentOutput, -speed);
			kRearRightChannel.set(ControlMode.PercentOutput, speed);
			kFrontRightChannel.set(ControlMode.PercentOutput, speed);
		}
	}

	public void smartDashboardEncoders() {
		/* get the decoded pulse width encoder position, 4096 units per rotation */
		int leftPulseWidthPos = kRearLeftChannel.getSensorCollection().getPulseWidthPosition();
		int rightPulseWidthPos = kRearRightChannel.getSensorCollection().getPulseWidthPosition();
		/* get measured velocity in units per 100ms, 4096 units is one rotation */
		int leftPulseWidthVel = kRearLeftChannel.getSensorCollection().getPulseWidthVelocity();
		int rightPulseWidthVel = kRearRightChannel.getSensorCollection().getPulseWidthVelocity();

		SmartDashboard.putNumber("Left Encoder Position", getLeftEncoderValue());
		SmartDashboard.putNumber("Right Encoder Position", getRightEncoderValue());
		SmartDashboard.putNumber("Left Encoder Velocity", leftPulseWidthVel);
		SmartDashboard.putNumber("Right Encoder Velocity", rightPulseWidthVel);

		SmartDashboard.putNumber("Gyro Angle:", gyro.getAngle());
	}

	public boolean checkIfNotDone(double distance, double time) {
		if (usingEncoders.getSelected()
				&& Math.abs(getLeftEncoderValue()) >= (distance / 18.85) * 4096) {
			return false;
		} else if (!usingEncoders.getSelected() && autoTimer.get() >= time) {
			return false;
		}
		return true;
	}

	public double getLeftEncoderValue() {
		
		return -(kRearLeftChannel.getSensorCollection().getPulseWidthPosition() - leftEncoderOffset);
	}
	
	public double getRightEncoderValue() {
		
		return (kRearRightChannel.getSensorCollection().getPulseWidthPosition() - rightEncoderOffset);
	}
	
	
	
	
	
	
}
