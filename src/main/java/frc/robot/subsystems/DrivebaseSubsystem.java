package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Drive;

public class DrivebaseSubsystem extends SubsystemBase {
  private final DifferentialDrive m_drive;

  private final Encoder m_leftDriveEncoder;
  private final Encoder m_rightDriveEncoder;

  private double m_y = 0;
  private double m_x = 0;

  private double m_scale = 1;

  private final PIDController m_leftDrivePID = new PIDController(Drive.kP, Drive.kI, Drive.kD);
  private final PIDController m_rightDrivePID = new PIDController(Drive.kP, Drive.kI, Drive.kD);

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Drive.TRACK_WIDTH);

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  MotorControllerGroup leftDriveGroup;
  MotorControllerGroup rightDriveGroup;

  /** Creates a new DrivebaseSubsystem. */
  public DrivebaseSubsystem(
    int leftDriveLeader, int leftDriveFollower,
    int rightDriveLeader, int rightDriveFollower,
    int leftDriveEncoder1, int leftDriveEncoder2,
    int rightDriveEncoder1, int rightDriveEncoder2
  ) {
    leftDriveGroup = new MotorControllerGroup(
      new CANSparkMax(leftDriveLeader, MotorType.kBrushless),
      new CANSparkMax(leftDriveFollower, MotorType.kBrushless)
    );
    rightDriveGroup = new MotorControllerGroup(
      new CANSparkMax(rightDriveLeader, MotorType.kBrushless),
      new CANSparkMax(rightDriveFollower, MotorType.kBrushless)
    );
    m_drive = new DifferentialDrive(leftDriveGroup, rightDriveGroup);
    m_leftDriveEncoder = new Encoder(leftDriveEncoder1, leftDriveEncoder2);
    m_rightDriveEncoder = new Encoder(rightDriveEncoder1, rightDriveEncoder2);
    // when robot goes forward, left encoder spins positive and right encoder spins negative
    m_leftDriveEncoder.setDistancePerPulse(Drive.DISTANCE_PER_ENCODER_PULSE);
    m_rightDriveEncoder.setDistancePerPulse(-Drive.DISTANCE_PER_ENCODER_PULSE);
    m_leftDriveEncoder.reset();
    m_rightDriveEncoder.reset();
  }

  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput =
            m_leftDrivePID.calculate(m_leftDriveEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput =
            m_rightDrivePID.calculate(m_rightDriveEncoder.getRate(), speeds.rightMetersPerSecond);
    leftDriveGroup.setVoltage(leftOutput + leftFeedforward);
    rightDriveGroup.setVoltage(rightOutput + rightFeedforward);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot Angular velocity in rad/s.
   */
  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
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
    return m_leftDriveEncoder.getDistance();
  }

  /**
   * Gets the distance travelled by the right-side wheels of the drivebase since last reset.
   * @return Distance in inches.
   */
  public double getRDistance() {
    return m_rightDriveEncoder.getDistance();
  }

  /**
   * Gets the speed of the left-side wheels of the drivebase.
   * @return Speed in inches / second.
   */
  public double getLRate() {
    return m_leftDriveEncoder.getRate();
  }

  /**
   * Gets the speed of the left-side wheels of the drivebase.
   * @return Speed in inches / second.
   */
  public double getRRate() {
    return m_rightDriveEncoder.getRate();
  }

  /**
   * Gets whether the drivebase is currently stopped.
   * @return true if stopped, false if moving.
   */
  public boolean getStopped() {
    return m_leftDriveEncoder.getStopped() && m_rightDriveEncoder.getStopped();
  }

  /** Resets drivebase encoder distances to 0. */
  public void resetEncoders() {
    m_leftDriveEncoder.reset();
    m_rightDriveEncoder.reset();
  }

  @Override
  public void periodic() {
    m_drive.curvatureDrive(m_x*m_scale, m_y*m_scale, true);
  }
}