//Robot-Code-2018 from Missfits github Acc
package org.usfirst.frc.team6418.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
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


enum StartingPosition {
	LEFT, MIDDLE, RIGHT
}

enum AutoStrategy {
	STRAIGHT, ISWITCH, OSWITCH, SCALE, NOTHING
}

enum XBoxButtons {
	DONOTUSE, A, B, X, Y, LEFT_BUMPER, RIGHT_BUMPER, BACK, START
}

enum XBoxAxes {
	DONOTUSE, LEFT_Y, LEFT_TRIGGER, RIGHT_TRIGGER, DONOTUSE2, RIGHT_Y
}

public class Robot extends IterativeRobot {

	//Spark intakeRight = new Spark(0);
	//Spark intakeLeft = new Spark(1);
	VictorSP intakeRight = new VictorSP(0);
	VictorSP intakeLeft = new VictorSP(1);

	VictorSP climber1 = new VictorSP(2);
	VictorSP climber2 = new VictorSP(3);

	final DigitalInput elevatorGroundLimit = new DigitalInput(0);
	final DigitalInput elevatorMaxLimit = new DigitalInput(1);
	
	final AnalogInput elevatorPot = new AnalogInput(0);

	final TalonSRX kFrontLeftChannel = new TalonSRX(2);
	final TalonSRX kRearLeftChannel = new TalonSRX(3);
	final TalonSRX kFrontRightChannel = new TalonSRX(1);
	final TalonSRX kRearRightChannel = new TalonSRX(4);

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
	public DoubleSolenoid intakeTiltSolenoid = new DoubleSolenoid(4, 5);

	public Timer autoTimer = new Timer();
	public Timer solenoidTimer = new Timer();

	public int autoState = 0;
	public int teleopState = 0;

	public int switchIsLeftState;
	public int scaleIsLeftState;

	public static ADXRS450_Gyro gyro = new ADXRS450_Gyro();

	public boolean climberDeployed = false;

	SendableChooser<StartingPosition> startPosition = new SendableChooser<>();
	SendableChooser<Boolean> usingEncoders = new SendableChooser<>();
	SendableChooser<AutoStrategy> autoStrategy = new SendableChooser<>();
	SendableChooser<Boolean> speedCap = new SendableChooser<>();

	public int elevatorZone = 1;

	public double leftEncoderOffset;
	public double rightEncoderOffset;

	@Override
	public void robotInit() {

		kFrontLeftChannel.setInverted(true);
		kRearLeftChannel.setInverted(true);
		// TODO may need to change or remove to match the robot
		// do we have to change the elevator motor? we won't worry about it

		startPosition.addDefault("Left", StartingPosition.LEFT);
		startPosition.addObject("Middle", StartingPosition.MIDDLE);
		startPosition.addObject("Right", StartingPosition.RIGHT);
		SmartDashboard.putData("Starting Position", startPosition);

		usingEncoders.addDefault("Encoders", true);
		usingEncoders.addObject("Timer", false);
		SmartDashboard.putData("Using Encoders", usingEncoders);

		autoStrategy.addObject("Only Straight", AutoStrategy.STRAIGHT);
		autoStrategy.addObject("Switch", AutoStrategy.ISWITCH);
		autoStrategy.addDefault("Fancy Scale", AutoStrategy.SCALE);
		autoStrategy.addObject("Do Nothing", AutoStrategy.NOTHING);
		SmartDashboard.putData("Auto Strategy", autoStrategy);
		
		speedCap.addObject("Cap Speed", true);
		speedCap.addDefault("Nah Bro", false);
		SmartDashboard.putData("Cap Speed?", speedCap);

		// operating compressor 
		compressor.setClosedLoopControl(true);
		closeIntake();
	}

	@Override
	public void disabledInit() {
		//System.out.println("Disabled Init");
	}

	@Override
	public void disabledPeriodic() {
		/*System.out.println("Disabled Periodic");
		System.out.println(autoStrategy.getSelected());*/
	}

	@Override
	public void autonomousInit() {
		autoState = 0;
		switchIsLeftState = switchIsLeft();
		scaleIsLeftState = scaleIsLeft();
		leftEncoderOffset = kRearLeftChannel.getSensorCollection().getPulseWidthPosition();
		rightEncoderOffset = kRearRightChannel.getSensorCollection().getPulseWidthPosition();
		gyro.reset(); 
	}

	@Override
	public void autonomousPeriodic() {
		if (autoStrategy.getSelected() == AutoStrategy.ISWITCH) {

			if ((switchIsLeftState == 1 && startPosition.getSelected() == StartingPosition.LEFT)
					|| (switchIsLeftState == 0 && startPosition.getSelected() == StartingPosition.RIGHT)) {
				driveStraightSwitch();
			} else if (startPosition.getSelected() == StartingPosition.MIDDLE) {
				middleAuto();
			} else {
				driveStraightOnly();
			}
		} else if (autoStrategy.getSelected() == AutoStrategy.SCALE) {
			if ((scaleIsLeftState == 1 && startPosition.getSelected() == StartingPosition.LEFT)
					|| (scaleIsLeftState == 0 && startPosition.getSelected() == StartingPosition.RIGHT)) {
				fancyScale(true);
			} else if ((scaleIsLeftState == 1 && startPosition.getSelected() == StartingPosition.RIGHT)
					|| (scaleIsLeftState == 0 && startPosition.getSelected() == StartingPosition.LEFT)) {
				fancyScale(false);
			} else if (startPosition.getSelected() == StartingPosition.MIDDLE) {
				middleAuto();
			} else {
				driveStraightOnly();
			}
		} else if (autoStrategy.getSelected() == AutoStrategy.STRAIGHT) {
			driveStraightOnly();
		} else {
			stopDrive();
		}

		SmartDashboard.putNumber("Switch Left? (1 = true) ", switchIsLeftState);
		SmartDashboard.putNumber("Scale Left? (1 = true) ", scaleIsLeftState);
		SmartDashboard.putNumber("Auto State:", autoState);
		smartDashboardEncoders();
		SmartDashboard.putNumber("Potentiometer Output", elevatorPot.getValue());
	}

	@Override
	public void teleopInit() {
		climberSolenoid.set(DoubleSolenoid.Value.kForward);
		climberDeployed = false;
		teleopState = 0;
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		
		SmartDashboard.putNumber("Potentiometer Output", elevatorPot.getValue());

		boolean elevatorGroundLimitPressed = elevatorGroundLimit.get();
		boolean elevatorMaxLimitPressed = elevatorMaxLimit.get();

		double leftJoystickY = leftStick.getY();
		double rightJoystickX = rightStick.getX();
		double rightJoystickY = rightStick.getY();

		double elevatorJoystickY = getAxis(XBoxAxes.RIGHT_Y);
		double climberJoystickY = getAxis(XBoxAxes.LEFT_Y);
		// the xbox joystic is inverted (joystick down registers as positive AND the
		// spark is wired backwards so the
		// negatives cancell each other out
		// gonna have to put boolean to make sure climber code doesn't run unless at
		// right height

		tiltIntake();

		/*if (Math.abs(climberJoystickY) >= 0.2) {
			// let the climber move backwards, move DOWN
			// it's setting it to positive so positive is reeling it in.
			climber1.set(climberJoystickY);
			climber2.set(climberJoystickY);
		} else {
			climber1.set(0);
			climber2.set(0);
		}*/

		// for elevator RED is UP. Now pushing up on XBOX makes elevator drive up

		if (elevatorJoystickY > 0.1 && (!elevatorGroundLimitPressed && elevatorPot.getValue() >= 200)) {
			//elevatorMotor.set(elevatorJoystickY);
			climber1.set(elevatorJoystickY);
			climber2.set(elevatorJoystickY);
			SmartDashboard.putString("Driving the elevator", "DOWN");
		} else if (elevatorJoystickY < -0.1 && (!elevatorMaxLimitPressed && elevatorPot.getValue() <= 2615)) {
			//elevatorMotor.set(elevatorJoystickY);
			climber1.set(elevatorJoystickY);
			climber2.set(elevatorJoystickY);
			SmartDashboard.putString("Driving the elevator", "UP");
		} else {
			//elevatorMotor.set(0);
			climber1.set(0); 
			climber2.set(0);
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
		
		
		if(speedCap.getSelected()) {
			if(buttonIsPressed(XBoxButtons.Y)){
				stopDrive();
			}
			kFrontLeftChannel.set(ControlMode.PercentOutput, -rightJoystickX*0.3);
			kRearRightChannel.set(ControlMode.PercentOutput, -rightJoystickX*0.3);
			kFrontRightChannel.set(ControlMode.PercentOutput, rightJoystickX*0.3);
			kRearLeftChannel.set(ControlMode.PercentOutput, rightJoystickX*0.3);
			//speedCap tank drive
			if (Math.abs(rightJoystickX) > 0.4) {
				kFrontLeftChannel.set(ControlMode.PercentOutput, -rightJoystickX*0.3);
				kRearRightChannel.set(ControlMode.PercentOutput, -rightJoystickX*0.3);
				kFrontRightChannel.set(ControlMode.PercentOutput, rightJoystickX*0.3);
				kRearLeftChannel.set(ControlMode.PercentOutput, rightJoystickX*0.3);
				// speedCap strafing
			}
		}else { 
			kFrontLeftChannel.set(ControlMode.PercentOutput, leftJoystickY);
			kRearLeftChannel.set(ControlMode.PercentOutput, leftJoystickY);
			kFrontRightChannel.set(ControlMode.PercentOutput, rightJoystickY);
			kRearRightChannel.set(ControlMode.PercentOutput, rightJoystickY);
			// manual tank drive
			if (Math.abs(rightJoystickX) > 0.4) {
				kFrontLeftChannel.set(ControlMode.PercentOutput, -rightJoystickX);
				kRearRightChannel.set(ControlMode.PercentOutput, -rightJoystickX);
				kFrontRightChannel.set(ControlMode.PercentOutput, rightJoystickX);
				kRearLeftChannel.set(ControlMode.PercentOutput, rightJoystickX);
				// manual strafing
				
			}
		}
		
		SmartDashboard.putBoolean("Elevator Ground Limit Pressed", elevatorGroundLimitPressed);
		SmartDashboard.putBoolean("Elevator Max Limit Pressed", elevatorMaxLimitPressed);
		smartDashboardEncoders();

		if (buttonIsPressed(XBoxButtons.RIGHT_BUMPER)) {
			intakeSolenoid.set(DoubleSolenoid.Value.kForward);
		} else if (buttonIsPressed(XBoxButtons.LEFT_BUMPER)) {
			intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
		} else {
			intakeSolenoid.set(DoubleSolenoid.Value.kOff);
		}

		/*
		 * here: if the top limit switch has not been pressed yet, you can keep moving
		 * up if it is moving downward && the bottom limit switch has not been pressed
		 * do all this logic in the elevator subsystem code.
		 */

		if (buttonIsPressed(XBoxButtons.BACK)) {
			climberSolenoid.set(DoubleSolenoid.Value.kForward);
		} else if (buttonIsPressed(XBoxButtons.START)) {
			climberDeployed = true;
			climberSolenoid.set(DoubleSolenoid.Value.kReverse);
		}

//		if (buttonIsPressed(XBoxButtons.A)) {
//			teleopState = takeInCube(teleopState);
//			SmartDashboard.putNumber("Teleop State: ", teleopState);
//		} else if (teleopState > 2) {
//			teleopState = 0;
//		}

		if (buttonIsPressed(XBoxButtons.B)) {
			dropCube();
		}

		/*SmartDashboard.putNumber("Ultrasonic Raw Bits", ultrasonic.getValue());
		SmartDashboard.putNumber("Ultrasonic Average Bits", ultrasonic.getAverageValue());
		SmartDashboard.putNumber("Ultrasonic Raw Voltage", ultrasonic.getVoltage());
		SmartDashboard.putNumber("Ultrasonic Average Voltage", ultrasonic.getAverageVoltage());*/

		SmartDashboard.putBoolean("Compressor Switch", compressor.getPressureSwitchValue());

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

	// battery voltage compensation:
	public double getVoltageCompensationMultipler() {
		return 12.7 / RobotController.getBatteryVoltage();
	}

	public boolean checkIfNotTurnt(double angle) {
		return (Math.abs(gyro.getAngle()) < Math.abs(angle) * 0.9);
	}

	public void turnToAngle(double angle) {
		double speed = 0.3 * getVoltageCompensationMultipler();
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

	public boolean checkIfNotDoneMoving(double distance, double time) {
		if (usingEncoders.getSelected() && Math.abs(getLeftEncoderValue()) >= (distance / 18.85) * 4096) {
			return false;
		} else if (!usingEncoders.getSelected() && autoTimer.get() >= time) {
			return false;
		}
		return true;
	}
	
	public boolean checkIfNotDoneMovingBackwards(double distance, double time) {
		if (usingEncoders.getSelected() && getLeftEncoderValue() >= (distance / 18.85) * 4096) {
			return true;
		} else if (!usingEncoders.getSelected() && autoTimer.get() >= time) {
			return true;
		}
		return false;
	}
	
	
	

	public void stopDrive() {
		kFrontLeftChannel.set(ControlMode.PercentOutput, 0.0);
		kRearRightChannel.set(ControlMode.PercentOutput, 0.0);
		kFrontRightChannel.set(ControlMode.PercentOutput, 0.0);
		kRearLeftChannel.set(ControlMode.PercentOutput, 0.0);
	}

	public double getLeftEncoderValue() {
		return -(kRearLeftChannel.getSensorCollection().getPulseWidthPosition() - leftEncoderOffset);
	}

	public double getRightEncoderValue() {
		return (kRearRightChannel.getSensorCollection().getPulseWidthPosition() - rightEncoderOffset);
	}

	public int scaleIsLeft() {
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		if (gameData.length() > 0) {
			if (gameData.charAt(1) == 'L') {
				return 1;
			} else {
				return 0;
			}
		}
		return -1;
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

	public void driveForward(double speed) {
		speed *= getVoltageCompensationMultipler();
		kFrontLeftChannel.set(ControlMode.PercentOutput, speed);
		kRearLeftChannel.set(ControlMode.PercentOutput, speed);
		kFrontRightChannel.set(ControlMode.PercentOutput, speed);
		kRearRightChannel.set(ControlMode.PercentOutput, speed);
	}

	public void driveStraight(double speed) {
		double leftSpeed = speed * getVoltageCompensationMultipler();
		double rightSpeed = speed * getVoltageCompensationMultipler();
		if (gyro.getAngle() < 0) {
			leftSpeed *= (1 + Math.abs(gyro.getAngle()) * 0.05);
		} else if (gyro.getAngle() > 0) {
			rightSpeed *= (1 + gyro.getAngle() * 0.5);
		}
		if (leftSpeed >= 1.0)
			leftSpeed = 1;
		else if (leftSpeed <= -1.0)
			leftSpeed = -1;
		if (rightSpeed >= 1.0)
			rightSpeed = 1;
		else if (rightSpeed <= -1.0)
			rightSpeed = -1;
		kFrontLeftChannel.set(ControlMode.PercentOutput, leftSpeed);
		kRearLeftChannel.set(ControlMode.PercentOutput, leftSpeed);
		kFrontRightChannel.set(ControlMode.PercentOutput, rightSpeed);
		kRearRightChannel.set(ControlMode.PercentOutput, rightSpeed);

		SmartDashboard.putNumber("LEFT Speed Drive Straight", leftSpeed);
		SmartDashboard.putNumber("RIGHT Speed Drive Straight", rightSpeed);
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

	public void moveElevator(double speed) {
		// get returns true when pressed
		if (speed < 0 && !elevatorMaxLimit.get()) {
			elevatorMotor.set(speed);
			SmartDashboard.putString("Driving the elevator", "UP");
		} else if (speed > 0 && !elevatorGroundLimit.get()) {
			elevatorMotor.set(speed);
			SmartDashboard.putString("Driving the elevator", "DOWN");
		} else {
			elevatorMotor.set(0);
			SmartDashboard.putString("Driving the elevator", "OFF");
		}
	}

	public void tiltIntake() {
		if (elevatorGroundLimit.get() || buttonIsPressed(XBoxButtons.X)) {
			intakeTiltSolenoid.set(DoubleSolenoid.Value.kForward);
		} else {
			intakeTiltSolenoid.set(DoubleSolenoid.Value.kReverse);
		}
	}

	public int takeInCube(int modeState) {
		int pastState = modeState;
		runIntake(-0.5);
		switch (modeState) {
		case 0:
			openIntake();
			modeState++;
			break;
		case 1:
			if (autoTimer.get() < 1.0) {
				driveForward(-0.5);
			} else {
				stopDrive();
				modeState++;
			}
			break;
		case 2:
			closeIntake();
			modeState++;
			break;
		case 3:
			if (autoTimer.get() < 0.5) {
				driveForward(-0.5);
			} else {
				modeState++;
			}
			break;
		default:
			runIntake(0);
			break;
		}
		if (pastState != modeState) {
			runIntake(0);
			autoTimer.reset();
			autoTimer.start();
		}
		return modeState;
	}

	public void dropCube() {
		// unnecessary
	//	openIntake();
		runIntake(0.6);
		// positive is shooting out
	}
	
	public boolean checkIfTooLow(int targetHeight) {
		//had parameter boolean goingToSwitch
//		int switchHeight = 800;
//		int scaleHeight = 2620;
//		int correctOutput = goingToSwitch? switchHeight : scaleHeight;
//		if(correctOutput > elevatorPot.getValue()) {
			
		if(targetHeight > elevatorPot.getValue()) {
			return true;
		}
		return false;
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
			if (checkIfNotDoneMoving(65, 2.0)) {
				// robot is 3'3", 38 in, 99 cm
				driveForward(-0.5);
			} else {
				autoState++;
			}
			break;
		case 2:
			if (checkIfTooLow(1300)) {
				moveElevator(-0.75);
			} else {
				autoState++;
			}
			break;
		case 3:
			if (checkIfNotDoneMoving(15, 2.0)) {
				driveForward(-0.25);
			} else {
				autoState++;
			}
			break;
		case 4:
			if (autoTimer.get() < 1.0) {
				runIntake(0.7);
			} else {
				openIntake();
				autoState++;
			}
			break;
		case 5:
			if (checkIfNotDoneMoving(20, 1.0)) {
				driveForward(0.25);
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
			if (checkIfNotDoneMoving(85, 1.5)) {
				driveForward(-0.3);
			} else {
				stopDrive();
				autoTimer.reset();
				autoTimer.start();
				leftEncoderOffset = kRearLeftChannel.getSensorCollection().getPulseWidthPosition();
				rightEncoderOffset = kRearRightChannel.getSensorCollection().getPulseWidthPosition();
				autoState++;
			}
			break;
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
		// TODO: test left side
		int pastState = autoState;
		double angle1 = 0, angle2 = 0;
		double turntDistance = 0, distanceAfterTurn2 = 0;
		if (switchIsLeftState == -1)
			return;
		else if (switchIsLeftState == 1) {
			// is left TODO
			angle1 = -34.5;
			angle2 = 36;
			turntDistance = 72; // 108-36
			distanceAfterTurn2 = 13;
		} else if (switchIsLeftState == 0) {
			angle1 = 35;
			angle2 = -37;
			turntDistance = 70;
			distanceAfterTurn2 = 13;
		}

		switch (autoState) {
		case 0:
			autoTimer.reset();
			autoTimer.start();
			autoState++;
			break;
		case 1:
			if (checkIfNotDoneMoving(6, 2.0)) {
				// robot is 3'3", 38 in, 99 cm
				driveForward(-0.5);
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
			if (autoTimer.get() < 0.5) {
				stopDrive();
			} else {
				autoState++;
			}
			break;
		case 4:
			if (checkIfNotDoneMoving(turntDistance, 2.0)) {
				driveForward(-0.4);
				if (checkIfTooLow(1300)) {
					moveElevator(-0.85);
				} else {
					moveElevator(0);
				}
			} else {
				autoState++;
			}
			break;
		case 5:
			if (checkIfNotTurnt(angle2))
				turnToAngle(angle2);
			else {
				autoState++;
			}
			break;
		case 6:
			// 40/tan((32*pi/180))-40/tan(35*pi/180)+6
			// if (checkIfNotDone(6, 2.0)) {
			if (checkIfNotDoneMoving(distanceAfterTurn2, 2.0)) {
				// robot is 3'3", 38 in, 99 cm
				driveForward(-0.5);
			} else {
				autoState++;
			}
			break;
		case 7:

			if (autoTimer.get() < 1.0) {
				runIntake(0.7);
			} else {
				openIntake();
				autoState++;
			}
			break;
		case 8:
			// 40/tan((32*pi/180))-40/tan(35*pi/180)+6
			if (checkIfNotDoneMoving(6, 2.0)) {
				// robot is 3'3", 38 in, 99 cm
				driveForward(0.5);
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

	public void oppositeSideScale() {
		int pastState = autoState;
		double angle = 0;
		if (scaleIsLeftState == -1) {
			return;
		} else if (scaleIsLeftState == 1) {
			angle = 90;
		} else if (scaleIsLeftState == 0) {
			angle = -90;
		}
		switch (autoState) {
		case 0:
			if (checkIfNotDoneMoving(212.25, 2.0)) {
				driveForward(-0.5);
			} else {
				autoState++;
			}
			break;
		case 1:
			if (checkIfNotTurnt(angle)) {
				turnToAngle(angle);
			} else {
				autoState++;
			}
			break;
		case 2:
			if (checkIfNotDoneMoving(256.87, 2.0)) {
				driveForward(-0.5);
			} else {
				autoState++;
			}
			break;
		case 3:
			if (checkIfNotTurnt(-angle)) {
				turnToAngle(-angle);
			} else {
				autoState++;
			}
			break;
		case 4:
			if (checkIfNotDoneMoving(94.65, 2.0)) {
				driveForward(-0.5);
			} else {
				autoState++;
			}
			break;
		case 5:
			if (checkIfNotTurnt(-angle)) {
				turnToAngle(-angle);
			} else {
				openIntake();
				autoState++;
			}
			break;
		case 6:
			if (autoTimer.get() < 1.0) {
				runIntake(0.7);
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

	public void fancyScale(boolean sameSide) {

		int pastState = autoState;
		int sameSideAngle = 0, oppositeAngle1 = 0, oppositeAngle2 = 0;
		if (sameSide) {
			if (scaleIsLeftState == 1) {
				sameSideAngle = 20;
			} else if (scaleIsLeftState == 0) {
				sameSideAngle = -20;
			}
		} else {
			// not same side
			if (scaleIsLeftState == 0) {
				oppositeAngle1 = 89;
				oppositeAngle2 = -120;
			} else if (scaleIsLeftState == 1) {
				oppositeAngle1 = -89;
				oppositeAngle2 = 120;
			}
		}
		switch (autoState) {
		case 0:
			autoTimer.reset();
			autoTimer.start();
			autoState++;
			break;
		case 1:
			if (checkIfNotDoneMoving(182, 2.0)) {
				// 195 with 0.5
				// robot is 3'3", 38 in, 99 cm
				driveStraight(-0.8);
				if(sameSide) {
					if (checkIfTooLow(1500)) {
						moveElevator(-0.85);
					}else {
						moveElevator(0);
					}
				}
			} else {
				autoState++;
			}
			break;
		case 2:
			if (sameSide) {
				autoState = 6;
			} else {
				if (checkIfNotTurnt(oppositeAngle1))
					turnToAngle(oppositeAngle1);
				else {
					autoState++;
				}
			}
			break;
		case 3:
			if (autoTimer.get() < 3.5) {
				stopDrive();
			} else {
				autoState++;
			}
			break;
		case 4:
			if (checkIfNotDoneMoving(192, 2.0)) {
				driveForward(-0.8);
			} else {
				autoState++;
			}
			break;
		case 5:
			if (checkIfNotTurnt(oppositeAngle2)) {
				turnToAngle(oppositeAngle2);
				if (checkIfTooLow(2621)) {
					moveElevator(-0.85);
				}else {
					moveElevator(0);
				}
			} else {
				autoState = 7;
			}
			break;
		case 6:
			if (checkIfNotTurnt(sameSideAngle)) {
				turnToAngle(sameSideAngle);
				if (checkIfTooLow(2621)) {
					moveElevator(-0.85);
				}else {
					moveElevator(0);
				}
			} else {
				autoState++;
			}
			break;
		case 7:
			if (checkIfTooLow(2621)) {
				moveElevator(-0.9);
			} else {
				intakeTiltSolenoid.set(DoubleSolenoid.Value.kReverse);
				autoState++;
			}
			break;
		case 8:
			if (checkIfNotDoneMoving(36, 0.2)) {
				driveStraight(-0.25);
				
			} else {
				autoState++;
			}
			break;
		case 9:
			if (autoTimer.get() < 1.0) {
				runIntake(0.7);
			} else {
				autoState++;
			}
			break;
		case 10:
			if(checkIfNotDoneMovingBackwards(-12,1.0)) {
				driveForward(0.25);
			}else {
				autoState++;
			}
			break;
		case 11:
			if(checkIfNotDoneMovingBackwards(-72,5.0)) {
				driveForward(0.25);
				if(elevatorPot.getValue() > 1000) {
					moveElevator(0.5);
				}else {
					moveElevator(0);
				}
			}else{
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
}
