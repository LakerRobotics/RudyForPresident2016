package org.usfirst.frc.team5053.robot.Subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Talon;

public class Arm implements Subsystem {
	
	private Talon m_Arm;
	private AnalogPotentiometer m_StringPot;
	private PIDController m_PID;
	
	public Arm(Talon armTalon, AnalogPotentiometer armStringPot) {
		m_Arm = armTalon;
		m_StringPot = armStringPot;
		m_PID = new PIDController(40.0, 4.0, 0.0, m_StringPot, m_Arm);
	}
	
	public void EnablePID() {
		m_PID.enable();
	}
	public void DisablePID() {
		m_PID.disable();
	}
	public double GetPosition() {
		return m_StringPot.get();
	}
	public void SetTargetPosition(double target) {
		m_PID.setSetpoint(target);
	}
	public void SetTalonOutput(double speed) { 
		m_Arm.set(speed);
	}
}
