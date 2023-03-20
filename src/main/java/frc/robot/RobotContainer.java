// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ScoringMode;
import frc.robot.Constants.ScoringTarget;
import frc.robot.Constants.SideToTurn;
import frc.robot.Constants.LED.modes;
import frc.robot.commands.*;
import frc.robot.commands.autoSequences.*;
import frc.robot.subsystems.*;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

  // private final CameraSubsystem m_camera = new CameraSubsystem();

  SendableChooser<AutoOption> m_autoChooser = new SendableChooser<>();

  SimpleWidget AutoFailedWidget = dashboard.add("Auto status", false).withPosition(7, 0);

  public RobotContainer() {
    // m_led.setMode(Constants.LED.modes.Rainbow);

    m_autoChooser.setDefaultOption("Auto Balance", AutoOption.AutoBalance);
    m_autoChooser.addOption("Auto Balance - No Return", AutoOption.AutoBalanceNoReturn);
    m_autoChooser.addOption("Drive forward - Short", AutoOption.ShortDriveForward);
    m_autoChooser.addOption("Drive forward - Long", AutoOption.LongDriveForward);
    dashboard.add("Auto Routine", m_autoChooser).withSize(2, 1).withPosition(8, 0);

    m_drivebase.setDefaultCommand(new DriveCommand(m_driverController::getLeftY, m_driverController::getRightX, m_drivebase));

    subsystemsDashboard.add(m_drivebase);
    subsystemsDashboard.add(m_led);
    // subsystemsDashboard.add(m_camera);
    subsystemsDashboard.add(m_shooter);

    AutoFailedWidget.withProperties(Map.of("colorWhenFalse", "grey"));

    m_drivebase.setDrivebaseIdle(CANSparkMax.IdleMode.kBrake);

    configureBindings();
  }

  private void configureBindings() {
    final JoystickButton rightBumper = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
    final JoystickButton leftBumper = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
    final JoystickButton aButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
    final JoystickButton bButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);
    final JoystickButton xButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
    final JoystickButton yButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);
    final JoystickButton startButton = new JoystickButton(m_driverController, XboxController.Button.kStart.value);
    final JoystickButton backButton = new JoystickButton(m_driverController, XboxController.Button.kBack.value);

    // final Button dpadUp = new Button(() -> {return m_driverController.getPOV() == 0});

    final JoystickButton operator_aButton = new JoystickButton(m_operatorController, XboxController.Button.kA.value);
    final JoystickButton operator_bButton = new JoystickButton(m_operatorController, XboxController.Button.kB.value);
    final JoystickButton operator_xButton = new JoystickButton(m_operatorController, XboxController.Button.kX.value); 
    final JoystickButton operator_yButton = new JoystickButton(m_operatorController, XboxController.Button.kY.value); 
    final JoystickButton operator_startButton = new JoystickButton(m_operatorController, XboxController.Button.kStart.value); 
    final JoystickButton operator_backButton = new JoystickButton(m_operatorController, XboxController.Button.kBack.value); 
    final JoystickButton operator_leftBumper = new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value); 
    final JoystickButton operator_rightBumper = new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value); 

    final Trigger leftTriggerHalfPull = new Trigger(m_driverController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, 0.5, CommandScheduler.getInstance().getDefaultButtonLoop()));
    final Trigger righTriggerHalfPull = new Trigger(m_driverController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.5, CommandScheduler.getInstance().getDefaultButtonLoop()));    

    // TODO Write eject sequence
    rightBumper.onTrue(new ShootSequence(m_drivebase, m_shooter, m_led));
    xButton.onTrue(new InstantCommand(() -> m_shooter.setFlap(Value.kForward)));
    xButton.onFalse(new InstantCommand(() -> m_shooter.setFlap(Value.kReverse)));
    // yButton.whileTrue(new BucketToZero(m_shooter, 0.5));

    leftTriggerHalfPull.whileTrue(new HalfDriveCommand(m_driverController::getLeftY, m_driverController::getRightX, m_drivebase, SideToTurn.LEFT));
    righTriggerHalfPull.whileTrue(new HalfDriveCommand(m_driverController::getLeftY, m_driverController::getRightX, m_drivebase, SideToTurn.RIGHT));
    // bButton.whileTrue(new ShootAngle(m_drivebase, m_shooter, 0.5));

    operator_aButton.whileTrue(new ShootAngle(m_drivebase, m_shooter, 1));
    // operator_bButton.whileTrue(new ShootAngle(m_drivebase, m_shooter, .8, Constants.ScoringTarget.MID_CONE));
    operator_yButton.onTrue(new InstantCommand(() -> m_shooter.setScoringMode(ScoringMode.CONE)).andThen(new InstantCommand(() -> m_led.setMode(modes.BreathingYellow))));
    operator_xButton.onTrue(new InstantCommand(() -> m_shooter.setScoringMode(ScoringMode.CUBE)).andThen(new InstantCommand(() -> m_led.setMode(modes.BreathingMagenta))));
    operator_bButton.onTrue(new InstantCommand(() -> m_shooter.setScoringMode(ScoringMode.FLAT)).andThen(new InstantCommand(() -> m_led.setMode(modes.Blue))));
    operator_leftBumper.whileTrue(new SetLEDAfterShot(m_shooter, m_led).andThen(new BucketToZero(m_shooter, 1)));
    operator_rightBumper.onTrue(new InstantCommand(() -> m_led.setMode(modes.Rainbow)));
    // operator_xButton.onTrue
    // operator_startButton.onTrue(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Yellow)));
    // operator_backButton.onTrue(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Purple)));
    // operator_lButton.onTrue(new DriveDistance(m_drivebase, 0.2, -1));
    // operator_rButton.onTrue(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Rainbow)));
    
    
    SmartDashboard.putData("Set to Mid Cone", new InstantCommand(() -> m_drivebase.setShot(ScoringTarget.MID_CONE)));
    SmartDashboard.putData("Set to Mid Cube", new InstantCommand(() -> m_drivebase.setShot(ScoringTarget.MID_CUBE)));
    SmartDashboard.putData("Set to High Cube", new InstantCommand(() -> m_drivebase.setShot(ScoringTarget.CUBE)));

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
