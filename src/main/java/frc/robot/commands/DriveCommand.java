// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.DrivebaseSubsystem;

public class DriveCommand extends CommandBase {
    
    private final DrivebaseSubsystem m_DrivebaseSubsystem;
    private DoubleSupplier m_left;
    private DoubleSupplier m_right;

    public DriveCommand(DoubleSupplier left, DoubleSupplier right, DrivebaseSubsystem subsystem) {
        m_left = left;
        m_right = right;
        m_DrivebaseSubsystem = subsystem;
        addRequirements(m_DrivebaseSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_DrivebaseSubsystem.set(m_left.getAsDouble(), m_right.getAsDouble());
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
