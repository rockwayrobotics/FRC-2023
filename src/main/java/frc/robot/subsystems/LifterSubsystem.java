// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.Pneumatics;

public class LifterSubsystem extends SubsystemBase {
  private DoubleSolenoid m_clawPneumatics;

  private CANSparkMax m_clawElevator;

  private DigitalInput m_topLimitSwitch;
  private DigitalInput m_bottomLimitSwitch;

  private double m_elevatorSpeed;

  /** Creates a new ExampleSubsystem. */
  public LifterSubsystem(
    int clawExtend, int clawRetract, int clawElevator, int topLimitSwitch, int bottomLimitSwitch
  ) {
    m_clawPneumatics = new DoubleSolenoid(Pneumatics.PNEUMATICS_MODULE_TYPE, clawExtend, clawRetract);
    m_clawElevator = new CANSparkMax(clawElevator, MotorType.kBrushless);

    m_topLimitSwitch = new DigitalInput(topLimitSwitch);
    m_bottomLimitSwitch = new DigitalInput(bottomLimitSwitch);
  }

  public void openClaw() {
    m_clawPneumatics.set(DoubleSolenoid.Value.kForward);
  }

  public void closeClaw() {
    m_clawPneumatics.set(DoubleSolenoid.Value.kReverse);
  }

  public void toggleClaw() {
    if(m_clawPneumatics.get() == DoubleSolenoid.Value.kForward) {
      m_clawPneumatics.set(DoubleSolenoid.Value.kReverse);
    } else if(m_clawPneumatics.get() == DoubleSolenoid.Value.kReverse) {
      m_clawPneumatics.set(DoubleSolenoid.Value.kForward);
    } else if(m_clawPneumatics.get() == DoubleSolenoid.Value.kOff) {
      m_clawPneumatics.set(DoubleSolenoid.Value.kReverse);
      System.out.println("Initializing Solenoid");
    } else {
      System.out.println("An unknown solenoid state was encountered. Solenoid state:");
      System.out.println(m_clawPneumatics.get());
    }
  }

  public void moveElevator(double speed) {
    m_elevatorSpeed = speed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(m_elevatorSpeed > 0 && m_topLimitSwitch.get() || m_elevatorSpeed < 0 && m_bottomLimitSwitch.get()) {
      m_elevatorSpeed = 0;
    }
    m_clawElevator.set(m_elevatorSpeed);
  }
}
