package org.usfirst.frc.team5053.robot.Subsystems;

import edu.wpi.first.wpilibj.Solenoid;

public class Kicker implements Subsystem{

	private Solenoid m_Kicker;
	
	public Kicker(Solenoid kickerSolenoid) {
		m_Kicker = kickerSolenoid;
	}
	
	public void SetSolenoidState(boolean state) {
		m_Kicker.set(state);
	}
	public boolean GetSolenoidState() {
		return m_Kicker.get();
	}
}
