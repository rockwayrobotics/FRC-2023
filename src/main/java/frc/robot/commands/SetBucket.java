// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class SetBucket extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_ShooterSubsystem;

  private boolean finishedSetting = false;

  private final DoubleSolenoid.Value m_cylinder1State;
  private final DoubleSolenoid.Value m_cylinder2State;

  /**
   * Creates a new SetBucket.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetBucket(ShooterSubsystem subsystem, DoubleSolenoid.Value cylinder1State, DoubleSolenoid.Value cylinder2State) {
    m_ShooterSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    m_cylinder1State = cylinder1State;
    m_cylinder2State = cylinder2State;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShooterSubsystem.setBucketCylinders(m_cylinder1State, m_cylinder2State);

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
