package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivebaseSubsystem;

public class DriveTime extends CommandBase {
    
    private DrivebaseSubsystem m_drivebase;
    private double m_speed;
    private double m_time;
    private double m_timeElapsed;

    public DriveTime(DrivebaseSubsystem subsystem, double speed, double time) {
        m_drivebase = subsystem;
        m_speed = speed;
        m_time = time;
    
        addRequirements(m_drivebase);
    }

    @Override
    public void initialize() {
        // Resets encoder values to default
        m_drivebase.resetEncoders();
    }

    @Override
    public void execute() {
        // Sets the drivebase to go forward from the speed variable
        m_drivebase.set(m_speed, 0);

        m_timeElapsed++;
    }

    @Override
    public boolean isFinished() {
        return (m_timeElapsed >= m_time);
    }

    @Override
    public void end(boolean cancelled) {
      m_drivebase.stop(); // Resets the drivebase to 0, ends command
    }
}
