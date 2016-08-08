package org.usfirst.frc.team5053.robot.Subsystems;

import edu.wpi.first.wpilibj.Talon;

public class SubsystemTemplate implements Subsystem
{
	//Member variables! Will be made private *most* of the time.
	private Talon m_SubsystemMotor;
	
	//A Constructor
	public SubsystemTemplate()
	{
		/** I'm a Subsystem Constructor!
		 * 
		 */
	}
	//Overloaded Constructor
	public SubsystemTemplate(Talon motor)
	{
		/** I'm a Subsystem Constructor that takes a motor on initialization!
		 *  This is where you would assign motor to a member variable in the subsystem
		 */
		
		m_SubsystemMotor = motor;
	}
	
	//Generic Set Method
	void setMethod(double value)
	{
		/**
		 * This set method sets the speed of the motor to the value of the passed argument.
		 */
		m_SubsystemMotor.set(value);
	}
	
	//Generic Get Method
	double getMethod()
	{
		/**
		 * This get method returns the speed of the motor.
		 */
		return m_SubsystemMotor.getSpeed();
	}
}
