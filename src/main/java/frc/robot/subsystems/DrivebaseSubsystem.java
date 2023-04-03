//DRIVEBASE SUBSYSTEM
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;


import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.Constants.Drive;
import org.photonvision.EstimatedRobotPose;

import java.util.Optional;



public class DrivebaseSubsystem extends SubsystemBase {
  private double m_scale = 1;

  private double yawOffset;

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  
  private final WPI_VictorSPX m_leftDriveMotor1 = new WPI_VictorSPX(Constants.CAN.LEFT_DRIVE_MOTOR_1);
  private final CANSparkMax m_leftDriveMotor2 = new CANSparkMax(Constants.CAN.LEFT_DRIVE_MOTOR_2, MotorType.kBrushless);

  private final WPI_VictorSPX m_rightDriveMotor1 = new WPI_VictorSPX(Constants.CAN.RIGHT_DRIVE_MOTOR_1);
  private final CANSparkMax m_rightDriveMotor2 = new CANSparkMax(Constants.CAN.RIGHT_DRIVE_MOTOR_2, MotorType.kBrushless);

  private final MotorControllerGroup m_leftDrive = new MotorControllerGroup(m_leftDriveMotor1, m_leftDriveMotor2);
  private final MotorControllerGroup m_rightDrive = new MotorControllerGroup(m_rightDriveMotor1, m_rightDriveMotor2);

  private final Encoder m_leftDriveEncoder = new Encoder(Constants.Digital.LEFT_DRIVE_ENCODER[0], Constants.Digital.LEFT_DRIVE_ENCODER[1]);
  private final Encoder m_rightDriveEncoder = new Encoder(Constants.Digital.RIGHT_DRIVE_ENCODER[0], Constants.Digital.RIGHT_DRIVE_ENCODER[1]);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftDrive, m_rightDrive);

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Drive.TRACK_WIDTH);
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(Drive.kS,Drive.kV);
  private final PIDController m_leftDrivePIDController = new PIDController(Drive.kP,Drive.kI,Drive.kD);
  private final PIDController m_rightDrivePIDController = new PIDController(Drive.kP,Drive.kI,Drive.kD);

  private final CameraSubsystem m_CameraSubsystem;

  private final DifferentialDrivePoseEstimator m_poseEstimator;

  /** Creates a new DrivebaseSubsystem. */
  public DrivebaseSubsystem(
    CameraSubsystem cameraSubsystem
  ) {
    m_CameraSubsystem = cameraSubsystem;

    m_leftDrive.setInverted(Constants.Drive.LEFT_DRIVE_INVERTED);
    m_rightDrive.setInverted(Constants.Drive.RIGHT_DRIVE_INVERTED);

    setDrivebaseIdle(IdleMode.kBrake);

    // when robot goes forward, left encoder spins positive and right encoder spins negative
    m_leftDriveEncoder.setDistancePerPulse(Drive.DISTANCE_PER_ENCODER_PULSE_METERS);
    m_rightDriveEncoder.setDistancePerPulse(Drive.DISTANCE_PER_ENCODER_PULSE_METERS);
    m_leftDriveEncoder.setReverseDirection(Constants.Drive.LEFT_DRIVE_INVERTED);
    m_rightDriveEncoder.setReverseDirection(Constants.Drive.RIGHT_DRIVE_INVERTED);

    m_leftDriveEncoder.reset();
    m_rightDriveEncoder.reset();
    
    m_poseEstimator = new DifferentialDrivePoseEstimator(m_kinematics, m_gyro.getRotation2d(), m_leftDriveEncoder.getDistance(), m_leftDriveEncoder.getDistance(), new Pose2d());
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
  // Returns yaw value from -180 to 180
  public double getYaw() {
    return m_gyro.getYaw() + yawOffset;
  }
  public double getPitch() {
    return m_gyro.getPitch();
  }
  public double getRoll() {
    return m_gyro.getRoll();
  }
  // Returns continuous yaw value
  public double getAngle() {
    return m_gyro.getAngle();
  }

  public void setDrivebaseIdle(IdleMode setting) {
    NeutralMode neutralMode = switch (setting) {
      case kBrake -> NeutralMode.Brake;
      case kCoast -> NeutralMode.Coast;
    };
    m_rightDriveMotor1.setNeutralMode(neutralMode);
    m_rightDriveMotor2.setIdleMode(setting);
    m_leftDriveMotor1.setNeutralMode(neutralMode);
    m_leftDriveMotor2.setIdleMode(setting);
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
    m_drive.curvatureDrive(speed*m_scale, speed*m_scale, true);
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
            m_leftDrivePIDController.calculate(m_leftDriveEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput =
            m_rightDrivePIDController.calculate(m_rightDriveEncoder.getRate(), speeds.rightMetersPerSecond);
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
   * Gets the average distance travelled between the two encoders since last reset
   * @return
   */
  public double getAverageDistance(){
    return (getRDistance() + getLDistance()) / 2;
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
  // public void resetEncoders() {
  //   m_leftDriveEncoder.reset();
  //   m_rightDriveEncoder.reset();
  // }

  public void updateOdometry(CameraSubsystem m_CameraSubsystem) {
    m_poseEstimator.update(m_gyro.getRotation2d(), m_leftDriveEncoder.getDistance(), m_rightDriveEncoder.getDistance());

    Optional<EstimatedRobotPose> result = m_CameraSubsystem.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());

    if (result.isPresent()) {
      EstimatedRobotPose camPose = result.get();
      m_poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
    }

    // System.out.println("Estimated pose: " + m_poseEstimator.getEstimatedPosition());
  }

  @Override
  public void periodic() {
    updateOdometry(m_CameraSubsystem);

    System.out.println(m_poseEstimator.getEstimatedPosition());

    SmartDashboard.putNumber("Right rate", getRRate());
    SmartDashboard.putNumber("Left rate", getLRate());

    SmartDashboard.putNumber("Gyro roll", getRoll());
    SmartDashboard.putNumber("Gyro Yaw", getYaw());
  }
}