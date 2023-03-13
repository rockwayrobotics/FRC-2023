// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ScoringTarget;
import frc.robot.commands.*;
import frc.robot.commands.autoSequences.*;
import frc.robot.subsystems.*;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.Map;

enum AutoOption {
  AutoBalance,
  AutoBalanceNoReturn,
  ShortDriveForward,
  LongDriveForward,
}

public class RobotContainer {
  ShuffleboardTab dashboard = Shuffleboard.getTab("Dashboard");
  ShuffleboardTab subsystemsDashboard = Shuffleboard.getTab("Subsystems");

  private final DrivebaseSubsystem m_drivebase = new DrivebaseSubsystem();

  private final LedSubsystem m_led = new LedSubsystem(Constants.LED.LED_PWM, 60);

  private final ShooterSubsystem m_shooter = new ShooterSubsystem();

  private final XboxController m_driverController = new XboxController(Constants.Gamepads.DRIVER);
  private final XboxController m_operatorController = new XboxController(Constants.Gamepads.OPERATOR);

  private final CameraSubsystem m_camera = new CameraSubsystem();

  SendableChooser<AutoOption> m_autoChooser = new SendableChooser<>();

  SimpleWidget AutoFailedWidget = dashboard.add("Auto status", false).withPosition(7, 0);

  public RobotContainer() {
    m_led.setMode(Constants.LED.modes.Rainbow);

    m_autoChooser.setDefaultOption("Auto Balance", AutoOption.AutoBalance);
    m_autoChooser.addOption("Auto Balance - No Return", AutoOption.AutoBalanceNoReturn);
    m_autoChooser.addOption("Drive forward - Short", AutoOption.ShortDriveForward);
    m_autoChooser.addOption("Drive forward - Long", AutoOption.LongDriveForward);
    dashboard.add("Auto Routine", m_autoChooser).withSize(2, 1).withPosition(8, 0);

    m_drivebase.setDefaultCommand(new DriveCommand(m_driverController::getLeftY, m_driverController::getRightX, m_drivebase));

    subsystemsDashboard.add(m_drivebase);
    subsystemsDashboard.add(m_led);
    subsystemsDashboard.add(m_camera);
    subsystemsDashboard.add(m_shooter);

    AutoFailedWidget.withProperties(Map.of("colorWhenFalse", "grey"));

    m_drivebase.setDrivebaseIdle(CANSparkMax.IdleMode.kBrake);

    configureBindings();
  }

  private void configureBindings() {
    final JoystickButton rightBumper = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
    final JoystickButton leftBumper = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
    final JoystickButton aButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
    final JoystickButton bButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);
    final JoystickButton xButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
    final JoystickButton yButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);
    final JoystickButton startButton = new JoystickButton(m_driverController, XboxController.Button.kStart.value);
    final JoystickButton backButton = new JoystickButton(m_driverController, XboxController.Button.kBack.value);

    final JoystickButton operator_aButton = new JoystickButton(m_operatorController, XboxController.Button.kA.value);
    final JoystickButton operator_bButton = new JoystickButton(m_operatorController, XboxController.Button.kB.value);
    final JoystickButton operator_yButton = new JoystickButton(m_operatorController, XboxController.Button.kY.value); 

    rightBumper.onTrue(new InstantCommand(() -> m_drivebase.setScale(Constants.Drive.SLOMODE_SCALE)));
    rightBumper.onFalse(new InstantCommand(() -> m_drivebase.setScale(1)));
    leftBumper.onTrue(new InstantCommand(() -> m_drivebase.setDrivebaseIdle(CANSparkMax.IdleMode.kCoast)));
    leftBumper.onFalse(new InstantCommand(() -> m_drivebase.setDrivebaseIdle(CANSparkMax.IdleMode.kBrake)));

    // TODO Write eject sequence
    aButton.onTrue(new ShootSequence(m_drivebase, m_shooter, m_led));
    xButton.onTrue(new LoadPieceSequence(m_drivebase, m_shooter, m_led));
    xButton.onFalse(new InstantCommand(() -> m_shooter.setFlap(Value.kReverse)));
    yButton.whileTrue(new BucketToZero(m_shooter, 0.5));
    bButton.whileTrue(new ShootAngle(m_drivebase, m_shooter, 0.5));

    operator_aButton.whileTrue(new ShootAngle(m_drivebase, m_shooter, .8));
    operator_bButton.whileTrue(new EjectAngle(m_drivebase, m_shooter, .8));
    operator_yButton.whileTrue(new BucketToZero(m_shooter, 0.5));
    
    SmartDashboard.putData("Set to Mid Cone", new InstantCommand(() -> m_drivebase.setShot(ScoringTarget.MID_CONE)));
    SmartDashboard.putData("Set to Mid Cube", new InstantCommand(() -> m_drivebase.setShot(ScoringTarget.MID_CUBE)));
    SmartDashboard.putData("Set to High Cube", new InstantCommand(() -> m_drivebase.setShot(ScoringTarget.HIGH_CUBE)));

    SmartDashboard.putData("Bucket Forward", new InstantCommand(() -> m_shooter.setBucketCylinders(Value.kForward, Value.kForward)));
    SmartDashboard.putData("Bucket Reverse", new InstantCommand(() -> m_shooter.setBucketCylinders(Value.kReverse, Value.kReverse)));
    SmartDashboard.putData("Flap Forward", new InstantCommand(() -> m_shooter.setFlap(Value.kForward)));
    SmartDashboard.putData("Flap Reverse", new InstantCommand(() -> m_shooter.setFlap(Value.kReverse)));

    SmartDashboard.putData("Shoot Sequence", new ShootSequence(m_drivebase, m_shooter, m_led));

    backButton.whileTrue(new AutoBalance(m_drivebase));
    startButton.onTrue(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Rainbow)));
  }

  public Command getAutonomousCommand() {
    m_drivebase.zeroGyro();

    // The selected command will be run in autonomous
    return switch (m_autoChooser.getSelected()) {
      case AutoBalance -> new BalanceRoutine(m_drivebase, m_shooter, m_led, AutoFailedWidget);
      case AutoBalanceNoReturn -> new CommunityRoutine(m_drivebase, m_shooter, m_led, AutoFailedWidget);
      case ShortDriveForward -> new ShortDriveForwardAutoRoutine(m_drivebase, m_shooter, m_led, AutoFailedWidget);
      case LongDriveForward -> new LongDriveForwardAutoRoutine(m_drivebase, m_shooter, m_led, AutoFailedWidget);
    };
  }
}
