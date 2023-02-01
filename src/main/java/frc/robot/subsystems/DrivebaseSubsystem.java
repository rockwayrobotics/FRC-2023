//DRIVEBASE SUBSYSTEM
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.Constants.Drive;



public class DrivebaseSubsystem extends SubsystemBase {
  private final DifferentialDrive m_drive;

  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;

  private double m_y = 0;
  private double m_x = 0;

  private double m_scale = 1;

  private static final double kP = 1.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kF = 0.0;

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Drive.TRACK_WIDTH);
  private final DifferentialDrivePoseEstimator m_poseEstimator = new DifferentialDrivePoseEstimator(m_kinematics, m_gyro.getRotation2d(),0.0, 0.0, new Pose2d());

  /** Creates a new DrivebaseSubsystem. */
  public DrivebaseSubsystem(
    int leftMotor1, int leftMotor2,
    int rightMotor1, int rightMotor2,
    int leftEncoder1, int leftEncoder2,
    int rightEncoder1, int rightEncoder2
  ) {
    m_gyro.reset();

    MotorControllerGroup leftDrive = new MotorControllerGroup(
      new CANSparkMax(leftMotor1, MotorType.kBrushless),
      new CANSparkMax(leftMotor2, MotorType.kBrushless)
    );
    MotorControllerGroup rightDrive = new MotorControllerGroup(
      new CANSparkMax(rightMotor1, MotorType.kBrushless),
      new CANSparkMax(rightMotor2, MotorType.kBrushless)
    );
    m_drive = new DifferentialDrive(leftDrive, rightDrive);
    m_leftEncoder = new Encoder(leftEncoder1, leftEncoder2);
    m_rightEncoder = new Encoder(rightEncoder1, rightEncoder2);
    // when robot goes forward, left encoder spins positive and right encoder spins negative
    m_leftEncoder.setDistancePerPulse(Drive.DISTANCE_PER_ENCODER_PULSE);
    m_rightEncoder.setDistancePerPulse(-Drive.DISTANCE_PER_ENCODER_PULSE);
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Sets the speed of the drivebase.
   * @param y Y speed. -1 is full backwards, 1 is full forwards.
   * @param x X speed. -1 is full left, 1 is full right.
   * @param priority Priority for this action. Only the highest priority action is run each cycle.
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

  public void updateOdometry(CameraSubsystem m_CameraSubsystem) {
    m_poseEstimator.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

    Optional<EstimatedRobotPose> result = m_CameraSubsystem.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());
  }

  @Override
  public void periodic() {
    m_drive.curvatureDrive(m_x*m_scale, m_y*m_scale, true);
  }
}