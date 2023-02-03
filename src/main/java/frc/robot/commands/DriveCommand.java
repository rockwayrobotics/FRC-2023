// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.DrivebaseSubsystem;

public class DriveCommand extends CommandBase {
    
    private final DrivebaseSubsystem m_DrivebaseSubsystem;
    private final DoubleSupplier m_leftJoyYFeeder;
    private final DoubleSupplier m_rightJoyXFeeder;

    public DriveCommand(DoubleSupplier leftJoyYFeeder, DoubleSupplier rightJoyXFeeder, DrivebaseSubsystem subsystem) {
        m_DrivebaseSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);

        m_leftJoyYFeeder = leftJoyYFeeder;
        m_rightJoyXFeeder = rightJoyXFeeder;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_DrivebaseSubsystem.set(m_leftJoyYFeeder.getAsDouble(), m_rightJoyXFeeder.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_DrivebaseSubsystem.set(0,0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
