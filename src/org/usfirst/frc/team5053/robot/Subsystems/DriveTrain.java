package org.usfirst.frc.team5053.robot.Subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
/**
 * Drivetrain subsystem that extends the FRC RobotDrive class.
 * @author Colin Ross
 *
 */

public class DriveTrain extends RobotDrive
{
	/**
	 * Hello There! :3 There's only one constructor.
	 */
	
	private Encoder m_LeftEncoder;
	private Encoder m_RightEncoder;
	
	public DriveTrain(SpeedController leftMotor, SpeedController rightMotor) 
	{
		super(leftMotor, rightMotor);
	}
	public DriveTrain(SpeedController leftMotor, SpeedController rightMotor, Encoder leftEncoder, Encoder rightEncoder)
	{
		super(leftMotor, rightMotor);
		
		m_LeftEncoder = leftEncoder;
		m_RightEncoder = rightEncoder;
		
	}
	
	public double getLeftEncoderRate()
	{
		if (m_LeftEncoder != null)
		{
			return m_LeftEncoder.getRate();
		}
		else 
		{
			return 0.0;
		}
	}
	public double getRightEncoderRate()
	{
		if (m_RightEncoder != null)
		{
			return m_RightEncoder.getRate();
		}
		else
		{
			return 0.0;
		}
	}
	
	public double getLeftEncoderDistance()
	{
		if (m_LeftEncoder != null)
		{
			return m_LeftEncoder.getDistance();
		}
		else 
		{
			return 0.0;
		}
	}
	public double getRightEncoderDistance()
	{
		if (m_RightEncoder != null)
		{
			return m_RightEncoder.getDistance();
		}
		else 
		{
			return 0.0;
		}
	}


}
