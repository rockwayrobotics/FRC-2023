package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivebaseSubsystem;

public class DriveDistance extends CommandBase {
    
    private DrivebaseSubsystem m_drivebase;
    private DoubleSupplier m_speed;
    private DoubleSupplier m_distance;

    /**
     * A command that spins the wheels for a certain distance
     * @param subsystem Set this to m_drivebase
     * @param distance Set to distance in inches
     * @param speed Set to speed from -1 to 1 (must match sign of distance)
     */
    public DriveDistance(DrivebaseSubsystem subsystem, DoubleSupplier distance, DoubleSupplier speed) {

        m_drivebase = subsystem;
        m_speed = speed;
        m_distance = distance;
    
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
        m_drivebase.set(m_speed.getAsDouble(), 0);
    }

    @Override
    public boolean isFinished() {
        // This takes the values from encoder L and R and averages them out
        double averageDistance = (m_drivebase.getLDistance() + m_drivebase.getRDistance()) / 2; 
        // This will say to end when the encoders are equal with the distance we want

        if (m_speed.getAsDouble() < 0) {
        return averageDistance <= m_distance.getAsDouble();
        } else {
        return averageDistance >= m_distance.getAsDouble();  
        }
    }

    @Override
    public void end(boolean cancelled) {
      m_drivebase.set(0,0); // Resets the drivebase to 0, ends command
    }
}
