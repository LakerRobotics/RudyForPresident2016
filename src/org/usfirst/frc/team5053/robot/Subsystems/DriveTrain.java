package org.usfirst.frc.team5053.robot.Subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
/**
 * Drivetrain subsystem that extends the FRC RobotDrive class.
 * @author Colin Ross
 *
 */

public class DriveTrain extends RobotDrive implements Subsystem
{
	/**
	 * Hello There! : I'm the base constructor.
	 */
	
	private SpeedController m_LeftMotor;
	private SpeedController m_RightMotor;
	
	private Encoder m_LeftEncoder;
	private Encoder m_RightEncoder;
	
	private ADXRS450_Gyro m_Gyro;
	
	private PIDController m_PID;
	
	public DriveTrain(SpeedController leftMotor, SpeedController rightMotor) 
	{
		super(leftMotor, rightMotor);
		m_LeftMotor = leftMotor;
		m_RightMotor = rightMotor;
		
		m_LeftMotor.setInverted(true);
		m_RightMotor.setInverted(true);
	}
	public DriveTrain(SpeedController leftMotor, SpeedController rightMotor, Encoder leftEncoder, Encoder rightEncoder)
	{
		super(leftMotor, rightMotor);
		
		m_LeftMotor = leftMotor;
		m_RightMotor = rightMotor;
		
		m_LeftEncoder = leftEncoder;
		m_RightEncoder = rightEncoder;
		
		m_LeftMotor.setInverted(true);
		m_RightMotor.setInverted(true);
	}
	public DriveTrain(SpeedController leftMotor, SpeedController rightMotor, Encoder leftEncoder, Encoder rightEncoder, ADXRS450_Gyro Gyro)
	{
		super(leftMotor, rightMotor);
		
		m_LeftMotor = leftMotor;
		m_RightMotor = rightMotor;
		
		m_LeftMotor.setInverted(true);
		m_RightMotor.setInverted(true);
		
		m_LeftEncoder = leftEncoder;
		m_RightEncoder = rightEncoder;
		
		m_Gyro = Gyro;
		m_Gyro.setPIDSourceType(PIDSourceType.kDisplacement);
		
		m_PID = new PIDController(15.0, 0.05, 0.0, m_Gyro, m_LeftMotor);
	}
	public void rotateDriveTrain(double angle) {
		if(angle > 360 || angle < -360)
		{
			double targetAngle = m_Gyro.getAngle() + angle;
			m_PID.setSetpoint(targetAngle);
		}
	}
	public HashMap<String, Double> GetDashboardData() {
		return null;
		// TODO Auto-generated method stub
		
	}

}
