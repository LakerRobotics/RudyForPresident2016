package org.usfirst.frc.team5053.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Talon;

public class ShooterAim {

	private Talon m_ShooterAim;
	private DigitalInput m_ShooterHigh;
	private DigitalInput m_ShooterLow;
	
	public ShooterAim(Talon shooterAimTalon, DigitalInput high, DigitalInput low) {
		m_ShooterAim = shooterAimTalon;
		m_ShooterHigh = high;
		m_ShooterLow = low;
	}
	
	public boolean LimitSwitchHigh() {
		return m_ShooterHigh.get();
	}
	public boolean LimitSwitchLow() {
		return m_ShooterLow.get();
	}
	public void SetTalonOutput(double speed) {
		m_ShooterAim.set(speed);
	}
}