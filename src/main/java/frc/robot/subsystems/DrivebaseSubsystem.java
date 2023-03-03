package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants.Drive;

public class DrivebaseSubsystem extends SubsystemBase {
  MotorControllerGroup leftDrive;
  MotorControllerGroup rightDrive;

  private final DifferentialDrive m_drive;

  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private double m_rotation = 0;
  private double m_speed = 0;

  private double m_scale = 1;
  private int direction = 1;

  private double yawOffset;

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Drive.TRACK_WIDTH);
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(Drive.kS,Drive.kV);
  private final PIDController m_leftDrivePIDController = new PIDController(Drive.kP,Drive.kI,Drive.kD);
  private final PIDController m_rightDrivePIDController = new PIDController(Drive.kP,Drive.kI,Drive.kD);

  MotorControllerGroup m_leftDrive;
  MotorControllerGroup m_rightDrive;

  ShuffleboardTab PIDTab = Shuffleboard.getTab("PID");

  SimpleWidget leftRate;
  SimpleWidget rightRate;

  GenericEntry leftRateEntry;
  GenericEntry rightRateEntry;

  /** Creates a new DrivebaseSubsystem. */
  public DrivebaseSubsystem(
    int leftMotor1, int leftMotor2,
    int rightMotor1, int rightMotor2,
    int leftEncoder1, int leftEncoder2,
    int rightEncoder1, int rightEncoder2
  ) {
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
    m_rightEncoder.setDistancePerPulse(Drive.DISTANCE_PER_ENCODER_PULSE);
    m_rightEncoder.setReverseDirection(true);
    m_leftEncoder.reset();
    m_rightEncoder.reset();

    leftRate = PIDTab.add("Left Rate", getLRate());
    leftRateEntry = leftRate.getEntry();
    rightRate = PIDTab.add("Right Rate", getLRate());
    rightRateEntry = rightRate.getEntry();
  }

  public void calibrateGyro() {
    m_gyro.calibrate();
  }
  public void zeroGyro() {
    System.out.println("NavX Connected: " + m_gyro.isConnected());
    m_gyro.reset();
  }
  public void setAutoOffset(double autoOffset) {
    yawOffset = autoOffset;
  }
  public double getYaw() {
    return m_gyro.getYaw() + yawOffset;
  }
  public double getPitch() {
    return m_gyro.getPitch();
  }
  public double getRoll() {
    return m_gyro.getRoll();
  }
  public double getAngle() {
    return m_gyro.getAngle();
  }

  public void stop(){
    set(0,0);
  }

  /**
   * Sets the speed of the drivebase.
   * @param speed Linear speed of drivetrain. -1 is full backwards, 1 is full forwards.
   * @param rotation Rotation speed. -1 is full left, 1 is full right.
   */
  public void set(double speed, double rotation) {
    m_speed = speed;
    m_rotation = rotation;
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
    m_drive.curvatureDrive(m_speed*m_scale, m_rotation*m_scale, true);
    // System.out.println(m_gyro.getPitch() + " pitch");
    //  System.out.println(mP0_gyro.getRoll() + " roll");

    SmartDashboard.putNumber("Gyro roll", getRoll());
    SmartDashboard.putNumber("Gyro Yaw", getYaw());
    leftRateEntry.setDouble(getLRate());
    rightRateEntry.setDouble(getRRate());
  }
}