package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.Constants.Drive;

public class DrivebaseSubsystem extends SubsystemBase {
  MotorControllerGroup leftDrive;
  MotorControllerGroup rightDrive;

  private final DifferentialDrive m_drive;

  private final Encoder m_leftDriveEncoder;
  private final Encoder m_rightDriveEncoder;

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private double m_rotation = 0;
  private double m_speed = 0;

  private double m_scale = 1;
  private int direction = 1;

  private double yawOffset;

  public double balance_kP = Constants.Balance.kP;
  public double balance_kD = Constants.Balance.kD;

  CANSparkMax m_leftDriveMotor1;
  CANSparkMax m_leftDriveMotor2;
  CANSparkMax m_rightDriveMotor1;
  CANSparkMax m_rightDriveMotor2;  

  public double highCubeBackupDistanceInches = 0;
  public double midCubeBackupDistanceInches;
  public double midConeBackupDistanceInches = 12;
  public double loadBackupDistanceInches = 3;

  GenericEntry highCubeDistanceWidget;
  GenericEntry midCubeDistanceWidget;
  GenericEntry midConeDistanceWidget;
  GenericEntry loadBackupDistanceWidget;

  public Constants.ScoringTarget m_scoringTarget;

  SendableChooser<Constants.ScoringTarget> m_scoringSelector;

  /** Creates a new DrivebaseSubsystem. */
  public DrivebaseSubsystem() {
    m_leftDriveMotor1 = new CANSparkMax(Constants.CAN.LEFT_DRIVE_MOTOR_1, MotorType.kBrushless);
    m_leftDriveMotor2 = new CANSparkMax(Constants.CAN.LEFT_DRIVE_MOTOR_2, MotorType.kBrushless);
    leftDrive = new MotorControllerGroup(m_leftDriveMotor1,m_leftDriveMotor2);
    leftDrive.setInverted(Constants.Drive.LEFT_DRIVE_INVERTED);

    m_rightDriveMotor1 = new CANSparkMax(Constants.CAN.RIGHT_DRIVE_MOTOR_1, MotorType.kBrushless);
    m_rightDriveMotor2 = new CANSparkMax(Constants.CAN.RIGHT_DRIVE_MOTOR_2, MotorType.kBrushless);
    rightDrive = new MotorControllerGroup(m_rightDriveMotor1, m_rightDriveMotor2);
    rightDrive.setInverted(Constants.Drive.RIGHT_DRIVE_INVERTED);

    m_drive = new DifferentialDrive(leftDrive, rightDrive);
    setDrivebaseIdle(IdleMode.kBrake);
    m_leftDriveEncoder = new Encoder(Constants.Digital.LEFT_DRIVE_ENCODER[0], Constants.Digital.LEFT_DRIVE_ENCODER[1]);
    m_rightDriveEncoder = new Encoder(Constants.Digital.RIGHT_DRIVE_ENCODER[0], Constants.Digital.RIGHT_DRIVE_ENCODER[1]);
    // when robot goes forward, left encoder spins positive and right encoder spins negative
    m_leftDriveEncoder.setDistancePerPulse(Drive.DISTANCE_PER_ENCODER_PULSE);
    m_rightDriveEncoder.setDistancePerPulse(Drive.DISTANCE_PER_ENCODER_PULSE);
    m_rightDriveEncoder.setReverseDirection(true);
    m_leftDriveEncoder.reset();
    m_rightDriveEncoder.reset();

    ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning");

    tuningTab.addPersistent("Balance kP", balance_kP).withPosition(0,0);
    tuningTab.addPersistent("Balance kD", balance_kD).withPosition(1,0);

    m_scoringSelector = new SendableChooser<Constants.ScoringTarget>();
    m_scoringSelector.setDefaultOption("High Cube", Constants.ScoringTarget.HIGH_CUBE);
    m_scoringSelector.addOption("Mid Cube", Constants.ScoringTarget.MID_CUBE);
    m_scoringSelector.addOption("Mid Cone", Constants.ScoringTarget.MID_CONE);

    ShuffleboardTab dashboardTab = Shuffleboard.getTab("Dashboard");

    dashboardTab.add("Scoring selector", m_scoringSelector).withSize(2,1).withPosition(0, 0);
    highCubeDistanceWidget = dashboardTab.addPersistent("High Cube Distance (inch)", 0).withPosition(2, 0).withSize(2, 1).getEntry();
    midCubeDistanceWidget =  dashboardTab.addPersistent("Mid Cube Distance (inch)", 15).withPosition(0, 1).withSize(2, 1).getEntry();
    midConeDistanceWidget = dashboardTab.addPersistent("Mid Cone Distance (inch)", 12).withPosition(2, 1).withSize(2, 1).getEntry();
    loadBackupDistanceWidget = dashboardTab.addPersistent("Pickup Distance (inch)", 3).withPosition(0,3).withSize(2,1).getEntry();
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

  public void setDrivebaseIdle(IdleMode setting) {
    m_rightDriveMotor1.setIdleMode(setting);
    m_rightDriveMotor2.setIdleMode(setting);
    m_leftDriveMotor1.setIdleMode(setting);
    m_leftDriveMotor2.setIdleMode(setting);
  }

  /**
   * Drive specificed direction 
   */
  public void drive(double leftPercentPower, double rightPercentPower){
    leftDrive.set(direction * leftPercentPower);
    rightDrive.set(direction * rightPercentPower);
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
    m_drive.curvatureDrive(m_speed*m_scale, m_rotation*m_scale, true);
    // System.out.println(m_gyro.getPitch() + " pitch");
    //  System.out.println(mP0_gyro.getRoll() + " roll");

    SmartDashboard.putNumber("Gyro roll", getRoll());
    SmartDashboard.putNumber("Gyro Yaw", getYaw());

    m_scoringTarget = m_scoringSelector.getSelected();

    highCubeBackupDistanceInches = highCubeDistanceWidget.getDouble(0);
    midCubeBackupDistanceInches = midCubeDistanceWidget.getDouble(15);
    midConeBackupDistanceInches = midCubeDistanceWidget.getDouble(12);
    loadBackupDistanceInches = loadBackupDistanceWidget.getDouble(3);
  }
}