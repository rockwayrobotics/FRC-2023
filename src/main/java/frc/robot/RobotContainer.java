// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;

import com.fasterxml.jackson.databind.jsontype.PolymorphicTypeValidator.Validity;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  private final CameraSubsystem m_CameraSubsystem = new CameraSubsystem();

  private DrivebaseSubsystem m_drivebase = new DrivebaseSubsystem(m_CameraSubsystem);

  private LedSubsystem m_led = new LedSubsystem(LED.LED_PWM, 60);

  private ShooterSubsystem m_shooter = new ShooterSubsystem();

  private XboxController m_xboxController = new XboxController(Gamepads.XBOX);

  SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  public final GenericEntry autoSpeed;

  public final Command m_driveForever;
  public final Command m_driveForeverSlow;
  public final Command m_balanceRoutine;


  public RobotContainer() {
    var autoTab = Shuffleboard.getTab("Auto");
    autoSpeed = autoTab.addPersistent("Max Speed", 1).withPosition(2, 0).getEntry();

    m_balanceRoutine = new BalanceRoutine(m_drivebase);
    
    m_driveForever = new DriveDistance(m_drivebase, autoSpeed.getDouble(0.5), 100000000);
    m_driveForeverSlow = new DriveDistance(m_drivebase, 0.1, 100000000);

    m_autoChooser.setDefaultOption("Drive Forever", m_driveForever);
    m_autoChooser.addOption("Drive Forever Slow", m_driveForeverSlow);
    // m_autoChooser.addOption("Auto Balance", m_autoBalance);
    m_autoChooser.addOption("Auto Balance", m_balanceRoutine);
    autoTab.add("Auto Routine", m_autoChooser).withSize(2, 1).withPosition(0, 0);

    m_drivebase.setDefaultCommand(new DriveCommand(() -> m_xboxController.getLeftY(), () -> m_xboxController.getRightX(), m_drivebase));

    Shuffleboard.getTab("Subsystems").add(m_CameraSubsystem);
    Shuffleboard.getTab("Subsystems").add(m_drivebase);

    configureBindings();
  }

  private void configureBindings() {
    final JoystickButton rightBumper = new JoystickButton(m_xboxController, XboxController.Button.kLeftBumper.value);
    rightBumper.onTrue(new SetDriveScaleCommand(m_drivebase, Drive.SLOMODE_SCALE));
    rightBumper.onFalse(new SetDriveScaleCommand(m_drivebase, 1));

    final JoystickButton buttonA = new JoystickButton(m_xboxController, XboxController.Button.kA.value);
    // buttonA.whileTrue(new AlignRobotToTarget(m_drivebase, m_CameraSubsystem));
    buttonA.whileTrue(new DrivethAlign(m_drivebase, m_CameraSubsystem));
    final JoystickButton buttonB = new JoystickButton(m_xboxController, XboxController.Button.kB.value);
    buttonB.whileTrue(new DrivethAlign(m_drivebase, m_CameraSubsystem));

    final JoystickButton buttonX = new JoystickButton(m_xboxController, XboxController.Button.kX.value);
    buttonX.whileTrue(new TestMotorSpin(m_drivebase));
    final JoystickButton backButton = new JoystickButton(m_xboxController, XboxController.Button.kBack.value);
    backButton.onTrue(new AutoBalance(m_drivebase));
  }

  public Command getAutonomousCommand() {
    m_drivebase.zeroGyro();

    // The selected command will be run in autonomous
    return m_autoChooser.getSelected();
  }
}
