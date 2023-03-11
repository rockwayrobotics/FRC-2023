// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  XboxController m_XboxController = new XboxController(0);

  Compressor m_compressor = new Compressor(Constants.CAN.PNEUMATIC_HUB, PneumaticsModuleType.REVPH);

  DoubleSolenoid m_bucket1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Pneumatics.bucketForwards1, Constants.Pneumatics.bucketReverse1);
  DoubleSolenoid m_bucket2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Pneumatics.bucketForwards2, Constants.Pneumatics.bucketReverse2);
  DoubleSolenoid m_flap = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Pneumatics.flapForwards, Constants.Pneumatics.flapReverse);

  CANSparkMax m_angleMotor = new CANSparkMax(Constants.CAN.SHOOTER_ANGLE_MOTOR, MotorType.kBrushless);
  RelativeEncoder m_angleEncoder = m_angleMotor.getEncoder();
  
  DigitalInput m_bottomLimit = new DigitalInput(Constants.Digital.SHOOTER_BOTTOM_LIMIT);
  DigitalInput m_alternateLimit = new DigitalInput(5);

  public Constants.ScoringTarget m_scoringTarget;

  SendableChooser<Constants.ScoringTarget> m_scoringSelector;
  
  public boolean angleLimitPressed = false;

  public double highCubeBackupDistanceInches = 0;
  public double midCubeBackupDistanceInches = 15;
  public double midConeBackupDistanceInches = 12;
  public double loadBackupDistanceInches = 3;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_scoringSelector = new SendableChooser<Constants.ScoringTarget>();
    m_scoringSelector.setDefaultOption("High Cube", Constants.ScoringTarget.HIGH_CUBE);
    m_scoringSelector.addOption("Mid Cube", Constants.ScoringTarget.MID_CUBE);
    m_scoringSelector.addOption("Mid Cone", Constants.ScoringTarget.MID_CONE);

    ShuffleboardTab dashboardTab = Shuffleboard.getTab("Dashboard");

    dashboardTab.add("Scoring selector", m_scoringSelector).withSize(2,1).withPosition(0, 0);
    dashboardTab.addPersistent("High Cube Distance (inch)", highCubeBackupDistanceInches).withPosition(2, 0).withSize(2, 1);
    dashboardTab.addPersistent("Mid Cube Distance (inch)", midCubeBackupDistanceInches).withPosition(0, 1).withSize(2, 1);
    dashboardTab.addPersistent("Mid Cone Distance (inch)", midConeBackupDistanceInches).withPosition(2, 1).withSize(2, 1);
    dashboardTab.addPersistent("Pickup Distance (inch)", loadBackupDistanceInches).withPosition(0,3).withSize(2,1);
  }

  public void setBucketCylinders(Value cylinder1State, Value cylinder2State) {
    m_bucket1.set(cylinder1State);
    m_bucket2.set(cylinder2State);
  }

  public void setFlap(Value flapDirection) {
    m_flap.set(flapDirection);
  }

  public void spinAngleMotor(double speed) {
    if (angleLimitPressed && speed > 0){
      m_angleMotor.set(0);
    } else {
      m_angleMotor.set(speed);
    }
  }

  public double getAngleEncoder() {
    return m_angleEncoder.getPosition();
  }

  public void setAngleEncoderPosition(double position) {
    m_angleEncoder.setPosition(position);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    angleLimitPressed = !m_bottomLimit.get();
    if (angleLimitPressed){
      setAngleEncoderPosition(0);
    }

    if(m_XboxController.getRightTriggerAxis() > 0) {
      spinAngleMotor(m_XboxController.getRightTriggerAxis());
    } else if(m_XboxController.getLeftTriggerAxis() > 0) {
      spinAngleMotor(-m_XboxController.getLeftTriggerAxis());
    } else {
      spinAngleMotor(0);
    }

    m_scoringTarget = m_scoringSelector.getSelected();
  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

  }
}
