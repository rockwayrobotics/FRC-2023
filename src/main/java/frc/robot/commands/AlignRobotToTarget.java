// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;

import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AlignRobotToTarget extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivebaseSubsystem m_DrivebaseSubsystem;
  private final CameraSubsystem m_CameraSubsystem;

  private PIDController turnController;

  private SimpleWidget colorWidget;
  private GenericEntry colorWidgetEntry;

  private boolean foundTarget;
  private int cyclesEmpty;
  private boolean finishedRunning;

  double rotationSpeed;

  double previousTimestamp;

  /**
   * Creates a new AlignRobotToTarget.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlignRobotToTarget(DrivebaseSubsystem DrivebaseSubsystem, CameraSubsystem CameraSubsystem) {
    m_DrivebaseSubsystem = DrivebaseSubsystem;
    m_CameraSubsystem = CameraSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DrivebaseSubsystem);
    addRequirements(CameraSubsystem);

    final double ANGULAR_P = 0.1;
    final double ANGULAR_D = 0.0;

    turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

    // Create Boolean widget that displays the color
    colorWidget = Shuffleboard.getTab("Vision").add("LostTarget", false);
    colorWidget.withPosition(0, 4);
    colorWidget.withProperties(Map.of("colorWhenFalse", "black"));
    colorWidget.withProperties(Map.of("colorWhenTrue", "red"));
    colorWidgetEntry = colorWidget.getEntry();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    foundTarget = false;
    cyclesEmpty = 0;
    rotationSpeed = 0;
    
    finishedRunning = false;

    turnController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    var result = m_CameraSubsystem.camera.getLatestResult();

    var resultTimestamp = result.getTimestampSeconds();

    if(resultTimestamp != previousTimestamp && result.hasTargets()) {
      previousTimestamp = resultTimestamp;

      foundTarget = true;
        
      rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);

      m_DrivebaseSubsystem.set(0, rotationSpeed);
    } else {
      if(foundTarget) {
        m_DrivebaseSubsystem.set(0, rotationSpeed);
        cyclesEmpty++;

        if(cyclesEmpty >= 100) {
            colorWidgetEntry.setBoolean(true);
            finishedRunning = true;
        }
      } else {
        finishedRunning = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finishedRunning;
  }
}