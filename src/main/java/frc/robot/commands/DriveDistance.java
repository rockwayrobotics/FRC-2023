package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivebaseSubsystem;

public class DriveDistance extends CommandBase {
    
    private DrivebaseSubsystem m_drivebase;
    private DoubleSupplier m_speed;

    public DriveDistance(DrivebaseSubsystem subsystem, DoubleSupplier speed) {

        m_drivebase = subsystem;
        m_speed = speed;
    
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
        m_drivebase.driveRobot(m_speed.getAsDouble(), 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean cancelled) {
      m_drivebase.driveRobot(0,0); // Resets the drivebase to 0, ends command
    }
}
