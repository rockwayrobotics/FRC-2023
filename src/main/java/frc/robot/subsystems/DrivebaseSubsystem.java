package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.Constants.*;

public class DrivebaseSubsystem extends SubsystemBase {
  MotorControllerGroup m_leftDrive = new MotorControllerGroup(
      new CANSparkMax(Drivetrain.kLeftDriveMotor1, MotorType.kBrushless),
      new CANSparkMax(Drivetrain.kLeftDriveMotor2, MotorType.kBrushless)
  );
  MotorControllerGroup m_rightDrive = new MotorControllerGroup(
    new CANSparkMax(Drivetrain.kRightDriveMotor1, MotorType.kBrushless),
    new CANSparkMax(Drivetrain.kRightDriveMotor2, MotorType.kBrushless)
  );
  DifferentialDrive m_drive = new DifferentialDrive(m_leftDrive, m_rightDrive);
  Encoder m_leftEncoder = new Encoder(Drivetrain.kLeftDriveEncoder1, Drivetrain.kLeftDriveEncoder2, Drivetrain.kLeftDriveInverted) ;
  Encoder m_rightEncoder = new Encoder(Drivetrain.kRightEncoder1, Drivetrain.kRightEncoder2, Drivetrain.kRightDriveInverted);

  private double m_scale = 1;

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DrivebaseSubsystem. */
  public DrivebaseSubsystem() {
    m_leftDrive.setInverted(Constants.Drivetrain.kLeftDriveInverted);
    m_rightDrive.setInverted(Constants.Drivetrain.kRightDriveInverted);

    m_leftEncoder.setDistancePerPulse(Drivetrain.kDriveDistancePerRevolution);
    m_rightEncoder.setDistancePerPulse(Drivetrain.kDriveDistancePerRevolution);

    resetEncoders();

    m_odometry =
        new DifferentialDriveOdometry(
            m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  /**
   * Sets the speed of the drivebase.
   * @param fwd Forward/back speed. -1 is full backwards, 1 is full forwards.
   * @param rot Rotation speed. -1 is full left, 1 is full right.
   */
  public void driveRobot(double fwd, double rot) {
    m_drive.curvatureDrive(fwd*m_scale, rot*m_scale, true);
  }

    /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftDrive.setVoltage(leftVolts);
    m_rightDrive.setVoltage(rightVolts);
    m_drive.feed();
  }

  /**
   * Sets the scale for the drivebase. Speeds are multiplied by the scale before being sent to the motors.
   * @param scale New scale to multiply speed by.
   */
  public void setScale(double scale) {
    m_scale = scale;
  }

  /** Resets drivebase encoder distances to 0. */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

    /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
        m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }
}