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
 * 
 * 
 * 
 */



//Sensors are located and handled by the RobotSensorMap class


public class RobotControllerMap
{


	private final int leftDrivePWM = 0;
	private final int rightDrivePWM = 1;
	
	//Example
	private final int exampleMotorPWM = 2;
	
	
	private Talon m_LeftDrive;
	private Talon m_RightDrive;
	
	//Example
	private Talon m_ExampleMotor;
	
	/**
	 * Hello There! :þ There's only one constructor.
	 */
	public RobotControllerMap()
	{
		
		m_LeftDrive = new Talon(leftDrivePWM);
		m_RightDrive = new Talon(rightDrivePWM);
		
		//Example
		m_ExampleMotor = new Talon(exampleMotorPWM);
	}
	
	public Talon GetLeftDrive()
	{
		return m_LeftDrive;
	}
	public Talon GetRightDrive()
	{
		return m_RightDrive;
	}
	
	//Example
	public Talon GetExampleMotor()
	{
		return m_ExampleMotor;
	}
	
	
	
}
