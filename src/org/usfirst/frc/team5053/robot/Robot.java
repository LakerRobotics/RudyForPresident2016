
package org.usfirst.frc.team5053.robot;

import org.usfirst.frc.team5053.robot.RobotInterfaceMap.joystickType;
import org.usfirst.frc.team5053.robot.Subsystems.Arm;
import org.usfirst.frc.team5053.robot.Subsystems.DriveTrain;
import org.usfirst.frc.team5053.robot.Subsystems.Intake;
import org.usfirst.frc.team5053.robot.Subsystems.Kicker;
import org.usfirst.frc.team5053.robot.Subsystems.LeftShooter;
import org.usfirst.frc.team5053.robot.Subsystems.RightShooter;
import org.usfirst.frc.team5053.robot.Subsystems.ShooterAim;

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
	Arm m_Arm;
	LeftShooter m_LeftShooter;
	RightShooter m_RightShooter;
	Intake m_Intake;
	ShooterAim m_ShooterAim;
	Kicker m_Kicker;
	
	//Arm setpoints
	final double ARM_NEUTRAL	= 0.209;
	final double ARM_AUTO		= 0.300;
	final double ARM_BALL		= 0.327;
	final double ARM_LOW		= 0.352;
	
	//Intake speeds
	final double INTAKE_FAST	= -2000;
	final double INTAKE_SLOW	= -1500;
	final double INTAKE_REVERSE = 2000;
	
	//Shooter speeds
	final double SHOOTER_FAST	= 2000;
	final double SHOOTER_SLOW	= 1500;
	final double SHOOTER_INTAKE	= -2000;
	
	int m_AutoCase;

    public void robotInit()
    {
        /**
         * This function is run when the robot is first started up and should be
         * used for any initialization code.
         */
    	
    	m_RobotInterface = new RobotInterfaceMap(joystickType.XBOX, joystickType.JOYSTICK);
    	//m_RobotInterface = new RobotInterfaceMap();
    	
    	m_RobotControllers = new RobotControllerMap();
    	m_RobotSensors = new RobotSensorMap();
    	
    	
    	//Robot Subsystem Initialization
    	m_DriveTrain = new DriveTrain(m_RobotControllers.GetLeftDrive(), m_RobotControllers.GetRightDrive(), m_RobotSensors.GetLeftDriveEncoder(), m_RobotSensors.GetRightDriveEncoder(), m_RobotSensors.GetGyro());
    	m_Arm = new Arm(m_RobotControllers.GetArm(), m_RobotSensors.GetArmPot());
    	m_LeftShooter = new LeftShooter(m_RobotControllers.GetLeftShooter(), m_RobotSensors.GetLeftShooterEncoder());
    	m_RightShooter = new RightShooter(m_RobotControllers.GetRightShooter(), m_RobotSensors.GetRightShooterEncoder());
    	m_Intake = new Intake(m_RobotControllers.GetIntake());
    	m_ShooterAim = new ShooterAim(m_RobotControllers.GetShooterBattery(), m_RobotSensors.GetShooterHigh(), m_RobotSensors.GetShooterLow());
    	m_Kicker = new Kicker(m_RobotSensors.GetKickerSolenoid());
    	
    	m_DriveTrain.ResetEncoders();
    	m_DriveTrain.ResetAngle();
    	
    	m_AutoCase = 0;
    }

    public void autonomousInit() 
    {
    	   /**
         * This function is called once when autonomous begins
         */
    	m_AutoCase = 0;
    	m_DriveTrain.ResetEncoders();
    	m_DriveTrain.ResetAngle();
    }

    public void autonomousPeriodic()
    {

        /**
         * This function is called periodically during autonomous
         */
    	switch (m_AutoCase)
    	{
    	case 0:
    		m_DriveTrain.SetPIDSetpoint(80, 0);
    		m_DriveTrain.EnablePID();
    		m_AutoCase++;
    		break;
    	case 1:
    		if (m_DriveTrain.DistanceOnTarget()){
    			m_DriveTrain.DisablePID();
    			m_AutoCase++;
    		}
    		break;
    	}
    	
    }


    public void teleopPeriodic()
    {
        /**
         * This function is called periodically during operator control
         */
    	ArcadeDrive();
    	
    	//Arm
    	if(m_RobotInterface.GetOperatorButton(1))
    	{
    		ManualArmControl();
    	}
    	else
    	{
    		ArmSetpoints();
    	}

    	//Intake
    	Intake();
    	
    	//Shooter
    	
    	
    	//Update Dashboard Variables
    	UpdateSmartDashboard();
    	
    	
    }

    public void testPeriodic()
    {
        
        /**
         * This function is called periodically during test mode
         */
    }
    //Drivetrain
    public void ArcadeDrive()
    {
    	m_DriveTrain.arcadeDrive(m_RobotInterface.GetDriverLeftY(), m_RobotInterface.GetDriverRightX());
    }
    
    //Arm
    public void ManualArmControl() 
    {
    	m_Arm.DisablePID();
        m_Arm.SetTalonOutput(-m_RobotInterface.GetOperatorJoystick().getRawAxis(1));
    }
    public void ArmSetpoints() 
    {	
    	if(!m_Arm.isPIDEnabled())
    	{
    		m_Arm.SetTalonOutput(0);
    	}
    	
    	if(m_RobotInterface.GetOperatorButton(7))
    	{
        	m_Arm.EnablePID();
    		m_Arm.SetTargetPosition(ARM_NEUTRAL);
    	}
    	else if(m_RobotInterface.GetOperatorButton(9))
    	{
    		m_Arm.EnablePID();
    		m_Arm.SetTargetPosition(ARM_BALL);
    	}
    	else if(m_RobotInterface.GetOperatorButton(11))
    	{
    		m_Arm.EnablePID();
    		m_Arm.SetTargetPosition(ARM_LOW);
    	}
    }
    
    //Intake
    public void Intake()
    {
	    if(m_RobotInterface.GetOperatorButton(8))
		{
			m_Intake.SetTalonOutput(INTAKE_FAST);

			m_LeftShooter.DisablePID();
			m_LeftShooter.SetTalonOutput(SHOOTER_INTAKE);
			
			m_RightShooter.DisablePID();
			m_RightShooter.SetTalonOutput(SHOOTER_INTAKE);
		}
		else if(m_RobotInterface.GetOperatorButton(10))
		{
			m_Intake.SetTalonOutput(INTAKE_SLOW);
			
			m_LeftShooter.DisablePID();
			m_LeftShooter.SetTalonOutput(SHOOTER_INTAKE);
			
			m_RightShooter.DisablePID();
			m_RightShooter.SetTalonOutput(SHOOTER_INTAKE);
		}
		else if(m_RobotInterface.GetOperatorButton(12))
		{
			m_Intake.SetTalonOutput(INTAKE_REVERSE);

			m_LeftShooter.DisablePID();
			m_LeftShooter.SetTalonOutput(SHOOTER_INTAKE);
			
			m_RightShooter.DisablePID();
			m_RightShooter.SetTalonOutput(SHOOTER_INTAKE);
		}
		else
		{
			m_Intake.SetTalonOutput(0);
			
			if(!m_LeftShooter.isPIDEnabled() && !m_RightShooter.isPIDEnabled())
			{
				m_LeftShooter.SetTalonOutput(0);
				m_RightShooter.SetTalonOutput(0);
			}
		}
    }
    
    //Shooter
    public void ShootFast()
    {
    	if(m_RobotInterface.GetOperatorButton(3))
    	{
    	   	m_LeftShooter.EnablePID();
        	m_RightShooter.EnablePID();
        	m_LeftShooter.SetShooterSetpoint(SHOOTER_FAST);
        	m_RightShooter.SetShooterSetpoint(SHOOTER_FAST);
        	if(m_LeftShooter.ShooterOnTarget() && m_RightShooter.ShooterOnTarget())
        	{
        		m_Kicker.SetSolenoidState(true);
        	}
    	}
    	else if(m_RobotInterface.GetOperatorButton(4))
    	{
    		m_LeftShooter.EnablePID();
        	m_RightShooter.EnablePID();
        	m_LeftShooter.SetShooterSetpoint(SHOOTER_SLOW);
        	m_RightShooter.SetShooterSetpoint(SHOOTER_SLOW);
        	if(m_LeftShooter.ShooterOnTarget() && m_RightShooter.ShooterOnTarget())
        	{
        		m_Kicker.SetSolenoidState(true);
        	}
    	}
    	else
    	{
    		m_LeftShooter.SetTalonOutput(0);
    		m_RightShooter.SetTalonOutput(0);
    	}
    }
    
    //Shoot aim | this code uses limit switches may want to revisit this and use PID control
    public void ShooterAim()
    {
    	boolean buttonPressed = false;
    	
    	while(m_RobotInterface.GetOperatorButton(12) || buttonPressed) {
    		buttonPressed = true;
    		if(m_ShooterAim.LimitSwitchLow())
    		{
    			m_ShooterAim.SetTalonOutput(.6);
    			if(m_ShooterAim.LimitSwitchHigh()) 
    			{
    				m_ShooterAim.SetTalonOutput(0);
    				buttonPressed = false;
    			}
    		}
    		else if(m_ShooterAim.LimitSwitchHigh()) 
    		{
    			m_ShooterAim.SetTalonOutput(-.6);
    			if(m_ShooterAim.LimitSwitchLow()) 
    			{
    				m_ShooterAim.SetTalonOutput(0);
    				buttonPressed = false;
    			}
    		}
    	}
    }
    public void UpdateSmartDashboard()
    {
    	SmartDashboard.putNumber("ArmPot", m_Arm.GetDashboardData().get("ArmPot"));
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
