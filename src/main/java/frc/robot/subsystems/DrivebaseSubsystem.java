package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.*;

public class DrivebaseSubsystem extends SubsystemBase {
  private final DifferentialDrive m_drive;

  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;

  private double m_y = 0;
  private double m_x = 0;

  private double m_scale = 1;

  /** Creates a new DrivebaseSubsystem. */
  public DrivebaseSubsystem() {
    MotorControllerGroup leftDrive = new MotorControllerGroup(
      new CANSparkMax(Drivetrain.kLeftDrive1, MotorType.kBrushless),
      new CANSparkMax(Drivetrain.kLeftDrive2, MotorType.kBrushless)
    );
    MotorControllerGroup rightDrive = new MotorControllerGroup(
      new CANSparkMax(Drivetrain.kRightDrive1, MotorType.kBrushless),
      new CANSparkMax(Drivetrain.kRightDrive2, MotorType.kBrushless)
    );
    m_drive = new DifferentialDrive(leftDrive, rightDrive);
    m_leftEncoder = new Encoder(Drivetrain.kLeftDriveEncoder1, Drivetrain.kLeftDriveEncoder2, Drivetrain.kLeftDriveEncoderInverted) ;
    m_rightEncoder = new Encoder(Drivetrain.kRightEncoder1, Drivetrain.kRightEncoder2, Drivetrain.kRightDriveEncoderInverted);
    // when robot goes forward, left encoder spins positive and right encoder spins negative
    m_leftEncoder.setDistancePerPulse(Drivetrain.kDriveDistancePerRevolution);
    m_rightEncoder.setDistancePerPulse(-Drivetrain.kDriveDistancePerRevolution);
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Sets the speed of the drivebase.
   * @param y Y speed. -1 is full backwards, 1 is full forwards.
   * @param x X speed. -1 is full left, 1 is full right.
   */
  public void set(double y, double x) {
      m_y = y;
      m_x = x;
  }

  /**
   * Sets the scale for the drivebase. Speeds are multiplied by the scale before being sent to the motors.
   * @param scale New scale to multiply speed by.
   */
  public void setScale(double scale) {
    m_scale = scale;
  }

  /**
   * Gets the distance travelled by the left-side wheels of the drivebase since last reset.
   * @return Distance, in inches.
   */
  public double getLDistance() {
    return m_leftEncoder.getDistance();
  }

  /**
   * Gets the distance travelled by the right-side wheels of the drivebase since last reset.
   * @return Distance in inches.
   */
  public double getRDistance() {
    return m_rightEncoder.getDistance();
  }

  /**
   * Gets the speed of the left-side wheels of the drivebase.
   * @return Speed in inches / second.
   */
  public double getLRate() {
    return m_leftEncoder.getRate();
  }

  /**
   * Gets the speed of the left-side wheels of the drivebase.
   * @return Speed in inches / second.
   */
  public double getRRate() {
    return m_rightEncoder.getRate();
  }

  /**
   * Gets whether the drivebase is currently stopped.
   * @return true if stopped, false if moving.
   */
  public boolean getStopped() {
    return m_leftEncoder.getStopped() && m_rightEncoder.getStopped();
  }

  /** Resets drivebase encoder distances to 0. */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  @Override
  public void periodic() {
    m_drive.curvatureDrive(m_x*m_scale, m_y*m_scale, true);
  }
}