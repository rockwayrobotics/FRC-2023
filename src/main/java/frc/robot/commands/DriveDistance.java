package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivebaseSubsystem;

public class DriveDistance extends CommandBase {
    
    private DrivebaseSubsystem m_drivebase;
    private double m_speed;
    private double m_distance;

    private double m_offset;

    public DriveDistance(DrivebaseSubsystem subsystem, double speed, double distance) {

        m_drivebase = subsystem;
        m_speed = speed;
        m_distance = distance;
    
        addRequirements(m_drivebase);
    }

    @Override
    public void initialize() {
        // Resets encoder values to default
        m_offset = m_drivebase.getAverageDistance();
    }

    @Override
    public void execute() {
        m_drivebase.set(m_speed, 0);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(m_drivebase.getAverageDistance() - m_offset) >= m_distance);
    }

    @Override
    public void end(boolean cancelled) {
        m_drivebase.stop(); // Resets the drivebase to 0, ends command
    }
}
