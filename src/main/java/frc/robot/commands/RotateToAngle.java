// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DrivebaseSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class RotateToAngle extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivebaseSubsystem m_DrivebaseSubsystem;

  private final PIDController pid = new PIDController(Constants.Drive.rotation_kP, 0, 0);

  private final double m_maxRotationPower;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RotateToAngle(DrivebaseSubsystem subsystem, double setpoint, double toleranceDegrees, double maxRotationPower) {
    m_DrivebaseSubsystem = subsystem;
    addRequirements(subsystem);

    pid.setSetpoint(setpoint);
    pid.setTolerance(toleranceDegrees);

    m_maxRotationPower = maxRotationPower;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double drivePower = pid.calculate(m_DrivebaseSubsystem.getYaw());

    drivePower = MathUtil.clamp(drivePower, -m_maxRotationPower, m_maxRotationPower);

    m_DrivebaseSubsystem.set(0, drivePower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      System.out.println("Cancelled when attempting to rotate to " + pid.getSetpoint() + " degrees");
    } else {
      System.out.println("Rotated to " + pid.getSetpoint() + " degrees");
    }
    m_DrivebaseSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
