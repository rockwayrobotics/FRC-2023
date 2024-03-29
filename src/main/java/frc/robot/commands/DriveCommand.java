// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.DrivebaseSubsystem;

public class DriveCommand extends CommandBase {
    
    private final DrivebaseSubsystem m_DrivebaseSubsystem;
    private DoubleSupplier m_left_y;
    private DoubleSupplier m_right_x;

    public DriveCommand(DoubleSupplier left_y, DoubleSupplier right_x, DrivebaseSubsystem subsystem) {
        m_left_y = left_y;
        m_right_x = right_x;
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
        double speed;
        double rotation;

        // TODO The comment below is incorrect. We need to test and see what direction does what.

        // Up is negative on joystick. When we push the stick up, we want the robot to move in a positive direction, so we invert it here.
        // Right is positive on the joystick. When we push the stick right, we want the robot to rotate clockwise (a negative direction), so we invert it here. 
        if(Math.abs(m_left_y.getAsDouble()) < 0.01) {
            speed = 0;
        } else {
            speed = m_left_y.getAsDouble();
        }

        if(Math.abs(m_right_x.getAsDouble()) < 0.01) {
            rotation = 0;
        } else {
            rotation = m_right_x.getAsDouble();
        }
        
        m_DrivebaseSubsystem.set(speed, -rotation*0.76);

        SmartDashboard.putNumber("Y", m_left_y.getAsDouble());
        SmartDashboard.putNumber("X", m_right_x.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_DrivebaseSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
