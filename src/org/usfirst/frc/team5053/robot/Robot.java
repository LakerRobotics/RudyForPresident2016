
package org.usfirst.frc.team5053.robot;

import org.usfirst.frc.team5053.robot.RobotInterfaceMap.joystickType;
import org.usfirst.frc.team5053.robot.Subsystems.DriveTrain;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * 
 * RUDY FOR PRESIDENT 2016
 * Laker Robotics 2016 Summer Rewrite
 * 
 */

public class Robot extends IterativeRobot
{

	/* Declare any and all variables here
	 *  Do not initialize until robotInit()
	 */
	
	//Robot Map Declaration
	RobotInterfaceMap m_RobotInterface;
	RobotControllerMap m_RobotControllers;
	RobotSensorMap m_RobotSensors;
	
	
	
	//Robot Subsystem Declaration
	DriveTrain m_DriveTrain;
	

    public void robotInit()
    {
        /**
         * This function is run when the robot is first started up and should be
         * used for any initialization code.
         */
    	
    	m_RobotInterface = new RobotInterfaceMap(joystickType.XBOX, joystickType.XBOX);
    	//m_robotInterface = new RobotInterfaceMap();
    	
    	m_RobotControllers = new RobotControllerMap();
    	m_RobotSensors = new RobotSensorMap();
    	
    	
    	
    	//Robot Subsystem Initialization
    	m_DriveTrain = new DriveTrain(m_RobotControllers.GetLeftDrive(), m_RobotControllers.GetRightDrive());
    }

    public void autonomousInit() 
    {
    	   /**
         * This function is called once when autonomous begins
         */
    }

    public void autonomousPeriodic()
    {

        /**
         * This function is called periodically during autonomous
         */
    }


    public void teleopPeriodic()
    {
        /**
         * This function is called periodically during operator control
         */
    	m_DriveTrain.arcadeDrive(m_RobotInterface.GetDriverJoystick());
    	
    	
    	
    	//Example Button Functionality
    	//Spin a motor while a button is pressed.
    	//In this case, while the A button on the driver controller is held down.
    	if (m_RobotInterface.GetDriverA())
    	{
    		spinMotor(m_RobotControllers.GetExampleMotor(), 0.5);
    	} 
    	else
    	{
    		stopMotor(m_RobotControllers.GetExampleMotor());
    	}
    	
    	//Update Dashboard Variables
    	UpdateSmartDashboard();
    	
    	
    }

    public void testPeriodic()
    {
        
        /**
         * This function is called periodically during test mode
         */
    }
    
    //Begin Laker Robotics Custom Methods
    public void spinMotor(SpeedController motor, double speed)
    {
    	motor.set(speed);
    }
    public void stopMotor(SpeedController motor)
    {
    	motor.stopMotor();
    }
    public void UpdateSmartDashboard()
    {
    	/* Not Implemented Yet
    	//Encoder Rates
    	SmartDashboard.putNumber("Left Encoder Rate", m_RobotSensors.getLeftEncoderRate());
    	SmartDashboard.putNumber("Right Encoder Rate", m_RobotSensors.getRightEncoderRate());
    	
    	//Encoder Distance
    	SmartDashboard.putNumber("Left Encoder Distance", m_RobotSensors.getLeftEncoderDistance());
    	SmartDashboard.putNumber("Right Encoder Distance", m_RobotSensors.getRightEncoderDistance());
    	*/
    }
    
}
