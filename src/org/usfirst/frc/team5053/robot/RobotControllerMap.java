package org.usfirst.frc.team5053.robot;

import edu.wpi.first.wpilibj.Talon;

/**
 * Maps all of the output controllers on the robot.
 * These include the following:
 * Talons,
 * Victors,
 * Jaguars,
 * Spikes,
 * Solenoids
 */



//Sensors are located and handled by the RobotSensorMap class


public class RobotControllerMap
{


	private final int leftDrivePWM = 0;
	private final int rightDrivePWM = 1;
	
	//Example
	private final int exampleMotorPWM = 2;
	
	
	private Talon m_leftDrive;
	private Talon m_rightDrive;
	
	//Example
	private Talon m_exampleMotor;
	
	/**
	 * Hello There! :3 There's only one constructor.
	 */
	public RobotControllerMap()
	{
		
		m_leftDrive = new Talon(leftDrivePWM);
		m_rightDrive = new Talon(rightDrivePWM);
		
		//Example
		m_exampleMotor = new Talon(exampleMotorPWM);
	}
	
	public Talon GetLeftDrive()
	{
		return m_leftDrive;
	}
	public Talon GetRightDrive()
	{
		return m_rightDrive;
	}
	
	//Example
	public Talon GetExampleMotor()
	{
		return m_exampleMotor;
	}
	
	
	
}
