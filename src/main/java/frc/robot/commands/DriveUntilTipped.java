// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivebaseSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveUntilTipped extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivebaseSubsystem m_DrivebaseSubsystem;

  private final double m_setpoint;
  private final double m_speed;

  /**
   * Creates a new DriveUntilTipped command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveUntilTipped(DrivebaseSubsystem subsystem, double setpoint, double speed) {
    m_DrivebaseSubsystem = subsystem;
    addRequirements(subsystem);

    m_setpoint = setpoint;
    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DrivebaseSubsystem.set(m_speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Reached tip setpoint of "+ m_setpoint + " degrees");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_setpoint - m_DrivebaseSubsystem.getRoll()) <= 3;
  }
}
