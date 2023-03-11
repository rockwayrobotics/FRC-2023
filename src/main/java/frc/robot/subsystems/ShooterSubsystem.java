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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

enum ShooterHeight {
  MID,
  HIGH,
}

enum GamePiece {
  CUBE,
  CONE,
}

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

  ShooterHeight m_height = ShooterHeight.MID;
  GamePiece m_piece = GamePiece.CUBE;

  SendableChooser<GamePiece> m_pieceSelector;
  SendableChooser<ShooterHeight> m_heightSelector;
  
  boolean angleLimitPressed = false; 

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_pieceSelector = new SendableChooser<GamePiece>();
    m_pieceSelector.setDefaultOption("Cube", GamePiece.CUBE);
    m_pieceSelector.addOption("Cone", GamePiece.CONE);
    Shuffleboard.getTab("Subsystems").add(m_pieceSelector);

    m_heightSelector = new SendableChooser<ShooterHeight>();
    m_heightSelector.setDefaultOption("Mid", ShooterHeight.MID);
    m_heightSelector.addOption("High", ShooterHeight.HIGH);
    Shuffleboard.getTab("Subsystems").add(m_heightSelector);
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
  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

  }
}
