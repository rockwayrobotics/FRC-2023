// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  XboxController m_DriverController = new XboxController(Constants.Gamepads.DRIVER);

  Compressor m_compressor = new Compressor(Constants.CAN.PNEUMATIC_HUB, PneumaticsModuleType.REVPH);

  DoubleSolenoid m_bucket1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Pneumatics.bucketForwards1, Constants.Pneumatics.bucketReverse1);
  DoubleSolenoid m_bucket2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Pneumatics.bucketForwards2, Constants.Pneumatics.bucketReverse2);
  DoubleSolenoid m_flap = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Pneumatics.flapForwards, Constants.Pneumatics.flapReverse);

  CANSparkMax m_angleMotor = new CANSparkMax(Constants.CAN.SHOOTER_ANGLE_MOTOR, MotorType.kBrushless);
  RelativeEncoder m_angleEncoder = m_angleMotor.getEncoder();
  
  DigitalInput m_bottomLimit = new DigitalInput(Constants.Digital.SHOOTER_BOTTOM_LIMIT);


  public double cubeAngleSetpoint;
  public double coneAngleSetpoint;
  public double ejectAngleSetpoint;


  GenericEntry cubeAngleWidget;
  GenericEntry coneAngleWidget;
  GenericEntry ejectAngleWidget;
  
  public boolean angleLimitPressed = false;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    ShuffleboardTab dashboardTab = Shuffleboard.getTab("Dashboard");

    cubeAngleWidget = dashboardTab.addPersistent("Cube angle", 266).withPosition(2,2).getEntry();
    coneAngleWidget = dashboardTab.addPersistent("Cone angle", 419).withPosition(3,2).getEntry();
    ejectAngleWidget = dashboardTab.addPersistent("Eject angle", 0).withPosition(2,3).getEntry();
  }

  public void setBucketCylinders(Value cylinder1State, Value cylinder2State) {
    m_bucket1.set(cylinder1State);
    m_bucket2.set(cylinder2State);
  }

  public void setFlap(Value flapDirection) {
    m_flap.set(flapDirection);
  }

  public void spinAngleMotor(double speed) {
    if (angleLimitPressed && Math.abs(speed) < 0){
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
    // System.out.println("Angle limit: " + m_bottomLimit.get());
    angleLimitPressed = !m_bottomLimit.get(); // Switch reads false when pressed
    if (angleLimitPressed){
      setAngleEncoderPosition(0);
    }

    // if(m_DriverController.getRightTriggerAxis() > 0) {
    //   spinAngleMotor(m_DriverController.getRightTriggerAxis());
    // } else if(m_DriverController.getLeftTriggerAxis() > 0) {
    //   spinAngleMotor(-m_DriverController.getLeftTriggerAxis());
    // } else {
    //   spinAngleMotor(0);
    // }

    SmartDashboard.putNumber("Encoder revolutions", getAngleEncoder());

    cubeAngleSetpoint = cubeAngleWidget.getDouble(266);
    coneAngleSetpoint = coneAngleWidget.getDouble(419);
    ejectAngleSetpoint = ejectAngleWidget.getDouble(0);
  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
