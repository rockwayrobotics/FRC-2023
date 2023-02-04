// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivebaseSubsystem;

import java.util.function.DoubleSupplier;

/** An example command that uses an example subsystem. */
public class PIDDriveCommand extends CommandBase {
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivebaseSubsystem m_DrivebaseSubsystem;
  private final DoubleSupplier m_leftJoyYFeeder;
  private final DoubleSupplier m_rightJoyXFeeder;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PIDDriveCommand(DoubleSupplier leftJoyYFeeder, DoubleSupplier rightJoyXFeeder, DrivebaseSubsystem subsystem) {
    m_DrivebaseSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    m_leftJoyYFeeder = leftJoyYFeeder;
    m_rightJoyXFeeder = rightJoyXFeeder;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double m_leftJoyY = m_leftJoyYFeeder.getAsDouble();
    if (Math.abs(m_leftJoyY) < 0.075) {
      m_leftJoyY = 0;
    }
    final var xSpeed = -m_speedLimiter.calculate(m_leftJoyY) * Constants.Drive.MAX_SPEED;

    double m_rightJoyX = m_rightJoyXFeeder.getAsDouble();
    if (Math.abs(m_rightJoyX) < 0.075) {
      m_rightJoyX = 0;
    }
    final var rotationSpeed = -m_rotLimiter.calculate(m_rightJoyX) * Constants.Drive.MAX_ROTATION_SPEED;

    m_DrivebaseSubsystem.drive(xSpeed, rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}