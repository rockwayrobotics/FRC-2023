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
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  Compressor m_compressor = new Compressor(1, PneumaticsModuleType.REVPH);

  DoubleSolenoid m_bucket1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Pneumatics.bucketForwards1, Constants.Pneumatics.bucketReverse1);
  DoubleSolenoid m_bucket2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Pneumatics.bucketForwards2, Constants.Pneumatics.bucketReverse2);

  CANSparkMax m_angleMotor = new CANSparkMax(Constants.CAN.ANGLE_MOTOR, MotorType.kBrushless);
  RelativeEncoder m_angleEncoder = m_angleMotor.getEncoder();
  
  DigitalInput m_angleSwitch = new DigitalInput(Constants.Digital.ANGLE_SWITCH);
  
  boolean angleLimitPressed = false; 

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {}

  public void setBucketCylinders(Value cylinder1State, Value cylinder2State) {
    m_bucket1.set(cylinder1State);
    m_bucket2.set(cylinder2State);
  }

  public void spinAngleMotor(double speed){
    if (angleLimitPressed){
      m_angleMotor.set(0);
    } else {
      m_angleMotor.set(speed);
    }
  }

  public double getAngleEncoder(){
    return m_angleEncoder.getPosition();
  }

  public void setAngleEncoderPosition(double position){
    m_angleEncoder.setPosition(position);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    angleLimitPressed = m_angleSwitch.get();
    if (angleLimitPressed){
      setAngleEncoderPosition(0);
    }
  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

  }
}
