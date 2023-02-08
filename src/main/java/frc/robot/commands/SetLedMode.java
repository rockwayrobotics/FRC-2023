// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.LedSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class SetLedMode extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LedSubsystem m_LedSubsystem;
  private final int m_mode;

  private boolean finishedSetting = false;

  /**
   * Creates a new SlowmodeCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetLedMode(LedSubsystem subsystem, int mode) {
    m_LedSubsystem = subsystem;
    m_mode = mode;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_LedSubsystem.setMode(m_mode);

    finishedSetting = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finishedSetting;
  }
}
