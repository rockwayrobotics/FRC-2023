// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  private DrivebaseSubsystem m_drivebase = new DrivebaseSubsystem(
    CAN.LEFT_MOTOR_1, CAN.LEFT_MOTOR_2,
    CAN.RIGHT_MOTOR_1, CAN.RIGHT_MOTOR_2,
    Digital.LEFT_ENCODER_1, Digital.LEFT_ENCODER_2,
    Digital.RIGHT_ENCODER_1, Digital.RIGHT_ENCODER_2
  );

  private LedSubsystem m_led = new LedSubsystem(LedConstant.LED_PWM, 60);

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

    Shuffleboard.getTab("Subsystems").add(m_drivebase);

    configureBindings();
  }

  private void configureBindings() {
    final JoystickButton rightBumper = new JoystickButton(m_xboxController, XboxController.Button.kLeftBumper.value);
    rightBumper.onTrue(new SetDriveScaleCommand(m_drivebase, Drive.SLOMODE_SCALE));
    rightBumper.onFalse(new SetDriveScaleCommand(m_drivebase, 1));
    final JoystickButton aButton = new JoystickButton(m_xboxController, XboxController.Button.kA.value);
    final JoystickButton bButton = new JoystickButton(m_xboxController, XboxController.Button.kB.value);
    final JoystickButton xButton = new JoystickButton(m_xboxController, XboxController.Button.kX.value);
    final JoystickButton yButton = new JoystickButton(m_xboxController, XboxController.Button.kY.value);
    final JoystickButton startButton = new JoystickButton(m_xboxController, XboxController.Button.kStart.value);
    final JoystickButton backButton = new JoystickButton(m_xboxController, XboxController.Button.kBack.value);
    aButton.onTrue(new SetLedMode(m_led, LedConstant.modes.Green));
    bButton.onTrue(new SetLedMode(m_led, LedConstant.modes.Red));
    xButton.onTrue(new SetLedMode(m_led, LedConstant.modes.Blue));
    yButton.onTrue(new SetLedMode(m_led, LedConstant.modes.Yellow));
    backButton.onTrue(new AutoBalance(m_drivebase));
    //backButton.and(aButton).onTrue(new InstantCommand(balance::cancel));
    startButton.onTrue(new SetLedMode(m_led, LedConstant.modes.Rainbow));
  }

  public Command getAutonomousCommand() {
    m_drivebase.zeroGyro();

    // The selected command will be run in autonomous
    return m_autoChooser.getSelected();
  }
}
