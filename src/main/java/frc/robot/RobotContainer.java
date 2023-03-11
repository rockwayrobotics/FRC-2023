// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import frc.robot.commands.*;
import frc.robot.commands.autoSequences.*;
import frc.robot.subsystems.*;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

enum AutoOption {
  AutoBalance,
  AutoBalanceNoReturn,
  DriveForward,
  AutoBalanceNoTurn,
  AutoBalanceNoTurnNoReturn,
}

public class RobotContainer {

  private final DrivebaseSubsystem m_drivebase = new DrivebaseSubsystem();

  private final LedSubsystem m_led = new LedSubsystem(Constants.LED.LED_PWM, 60);

  private final ShooterSubsystem m_shooter = new ShooterSubsystem();

  private final XboxController m_xboxController = new XboxController(Constants.Gamepads.XBOX);

  private final CameraSubsystem m_camera = new CameraSubsystem();

  SendableChooser<AutoOption> m_autoChooser = new SendableChooser<>();

  public final GenericEntry autoSpeed;

  public RobotContainer() {
    var autoTab = Shuffleboard.getTab("Auto");
    autoSpeed = autoTab.addPersistent("Max Speed", 1).withPosition(2, 0).getEntry();

    m_autoChooser.setDefaultOption("Auto Balance", AutoOption.AutoBalance);
    m_autoChooser.addOption("Auto Balance - No Return", AutoOption.AutoBalanceNoReturn);
    m_autoChooser.addOption("Drive forward", AutoOption.DriveForward);
    m_autoChooser.addOption("Auto Balance - No Turn", AutoOption.AutoBalanceNoTurn);
    m_autoChooser.addOption("Auto Balance - No Turn - No Return", AutoOption.AutoBalanceNoTurnNoReturn);
    autoTab.add("Auto Routine", m_autoChooser).withSize(2, 1).withPosition(0, 0);

    m_drivebase.setDefaultCommand(new DriveCommand(m_xboxController::getLeftY, m_xboxController::getRightX, m_drivebase));

    Shuffleboard.getTab("Subsystems").add(m_drivebase);

    configureBindings();
  }

  private void configureBindings() {
    final JoystickButton rightBumper = new JoystickButton(m_xboxController, XboxController.Button.kLeftBumper.value);
    final JoystickButton leftBumper = new JoystickButton(m_xboxController, XboxController.Button.kLeftBumper.value);
    final JoystickButton aButton = new JoystickButton(m_xboxController, XboxController.Button.kA.value);
    final JoystickButton bButton = new JoystickButton(m_xboxController, XboxController.Button.kB.value);
    final JoystickButton xButton = new JoystickButton(m_xboxController, XboxController.Button.kX.value);
    final JoystickButton yButton = new JoystickButton(m_xboxController, XboxController.Button.kY.value);
    final JoystickButton startButton = new JoystickButton(m_xboxController, XboxController.Button.kStart.value);
    final JoystickButton backButton = new JoystickButton(m_xboxController, XboxController.Button.kBack.value);

    rightBumper.onTrue(new InstantCommand(() -> m_drivebase.setScale(Constants.Drive.SLOMODE_SCALE)));
    rightBumper.onFalse(new InstantCommand(() -> m_drivebase.setScale(1)));
    leftBumper.onTrue(new InstantCommand(() -> m_drivebase.setDrivebaseIdle(CANSparkMax.IdleMode.kCoast)));
    leftBumper.onFalse(new InstantCommand(() -> m_drivebase.setDrivebaseIdle(CANSparkMax.IdleMode.kBrake)));
    aButton.onTrue(new InstantCommand(() -> m_shooter.setBucketCylinders(Value.kForward, Value.kForward)));
    bButton.onTrue(new InstantCommand(() -> m_shooter.setBucketCylinders(Value.kReverse, Value.kReverse)));
    xButton.onTrue(new InstantCommand(() -> m_shooter.setFlap(Value.kForward)));
    yButton.onTrue(new InstantCommand(() -> m_shooter.setFlap(Value.kReverse)));
    backButton.whileTrue(new AutoBalance(m_drivebase));
    startButton.onTrue(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Rainbow)));
  }

  public Command getAutonomousCommand() {
    m_drivebase.zeroGyro();

    // The selected command will be run in autonomous
    return switch (m_autoChooser.getSelected()) {
      case AutoBalance -> new BalanceRoutine(m_drivebase);
      case AutoBalanceNoReturn -> new CommunityRoutine(m_drivebase);
      case DriveForward -> new DriveForwardAutoRoutine(m_drivebase);
      case AutoBalanceNoTurn -> new NoTurnBalanceRoutine(m_drivebase);
      case AutoBalanceNoTurnNoReturn -> new NoTurnCommunityRoutine(m_drivebase);
    };
  }
}
