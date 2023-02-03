package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;

public class DrivebaseSubsystem extends SubsystemBase {
  private final DifferentialDrive m_drive;

  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;

  private double m_y = 0;
  private double m_x = 0;

  private double m_scale = 1;

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Drive.TRACK_WIDTH);
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(Drive.kS,Drive.kV);
  private final PIDController m_leftDrivePIDController = new PIDController(Drive.kP,Drive.kI,Drive.kD);
  private final PIDController m_rightDrivePIDController = new PIDController(Drive.kP,Drive.kI,Drive.kD);

  MotorControllerGroup m_leftDrive;
  MotorControllerGroup m_rightDrive;


  /** Creates a new DrivebaseSubsystem. */
  public DrivebaseSubsystem(
    int leftMotor1, int leftMotor2,
    int rightMotor1, int rightMotor2,
    int leftEncoder1, int leftEncoder2,
    int rightEncoder1, int rightEncoder2
  ) {
    m_gyro.reset();

    m_leftDrive = new MotorControllerGroup(
      new CANSparkMax(leftMotor1, MotorType.kBrushless),
      new CANSparkMax(leftMotor2, MotorType.kBrushless)
    );
    m_rightDrive = new MotorControllerGroup(
      new CANSparkMax(rightMotor1, MotorType.kBrushless),
      new CANSparkMax(rightMotor2, MotorType.kBrushless)
    );

    m_rightDrive.setInverted(true);

    m_drive = new DifferentialDrive(m_leftDrive, m_rightDrive);
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
   */
  public void set(double y, double x) {
      m_y = y;
      m_x = x;
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
            m_leftDrivePIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput =
            m_rightDrivePIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
    m_leftDrive.setVoltage(leftOutput + leftFeedforward);
    m_rightDrive.setVoltage(rightOutput + rightFeedforward);
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