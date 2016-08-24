package org.usfirst.frc.team5053.robot.Subsystems;

import edu.wpi.first.wpilibj.Talon;

public class Intake implements Subsystem{
	
	private Talon m_Intake;
	
	public Intake(Talon intakeTalon) {
		m_Intake = intakeTalon;
	}
	
	public void SetTalonOutput(double speed) {
		m_Intake.set(speed);
	}
}
