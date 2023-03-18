package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivebaseSubsystem;

public class ShootMoveDistance extends CommandBase {
    
    private final DrivebaseSubsystem m_drivebase;
    private final double m_speed;
    private double m_distance;

    public ShootMoveDistance(DrivebaseSubsystem subsystem, double speed) {

        m_drivebase = subsystem;
        m_speed = speed;
    
        addRequirements(m_drivebase);
    }

    @Override
    public void initialize() {
        // Resets encoder values to default
        m_drivebase.resetEncoders();
        
        switch (m_drivebase.m_scoringTarget) {
            case CUBE -> m_distance = m_drivebase.highCubeBackupDistanceInches;
            case MID_CUBE -> m_distance = m_drivebase.midCubeBackupDistanceInches;
            case MID_CONE -> m_distance = m_drivebase.midConeBackupDistanceInches;
        };

        System.out.println("Moving: " + m_distance);
    }

    @Override
    public void execute() {
        m_drivebase.set(m_speed, 0);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(m_drivebase.getRDistance()) >= m_distance);
    }

    @Override
    public void end(boolean cancelled) {
        System.out.println("Moved.");
        m_drivebase.stop(); // Resets the drivebase to 0, ends command
    }
}
