package org.usfirst.frc.team6418.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.SpeedController;

public class WPITalon implements SpeedController {
	private TalonSRX talon;
	
	public WPITalon(int deviceNumber) {
		talon = new TalonSRX(deviceNumber);
	}

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub

	}

	@Override
	public void set(double speed) {
		// TODO Auto-generated method stub
		talon.set(ControlMode.PercentOutput, speed);
	}

	@Override
	public double get() {
		// TODO Auto-generated method stub
		// return 0;
		return talon.getMotorOutputPercent();
	}

	@Override
	public void setInverted(boolean isInverted) {
		// TODO Auto-generated method stub
		talon.setInverted(isInverted);
	}

	@Override
	public boolean getInverted() {
		// TODO Auto-generated method stub
		return talon.getInverted();
	}

	@Override
	public void disable() {
		// TODO Auto-generated method stub
		talon.neutralOutput();
	}

	@Override
	public void stopMotor() {
		// TODO Auto-generated method stub
		talon.set(ControlMode.PercentOutput, 0.0);
	}

}
