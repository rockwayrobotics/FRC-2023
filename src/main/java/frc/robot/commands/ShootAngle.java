package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAngle extends CommandBase {

    private final ShooterSubsystem m_shooter;
    private final DrivebaseSubsystem m_drivebase;
    private final double m_speed;
    private double m_distance;

    public ShootAngle(DrivebaseSubsystem drivebase, ShooterSubsystem shooter, double speed) {

        m_shooter = shooter;
        m_drivebase = drivebase;
        m_speed = speed;
    
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        // Resets encoder values to default
        m_drivebase.resetEncoders();
        
        switch (m_drivebase.m_scoringTarget) {
            case HIGH_CUBE, MID_CUBE -> m_distance = m_shooter.cubeAngleSetpoint;
            case MID_CONE -> m_distance = m_shooter.coneAngleSetpoint;
        };

        System.out.println("Angling: " + m_distance);
    }

    @Override
    public void execute() {
        if(m_shooter.getAngleEncoder() > m_distance) {
            m_shooter.spinAngleMotor(m_speed);
        } else if(m_shooter.getAngleEncoder() < m_distance) {
            m_shooter.spinAngleMotor(-m_speed);
        }
    }

    @Override
    public boolean isFinished() {
        return (m_shooter.getAngleEncoder() == m_distance);
    }

    @Override
    public void end(boolean cancelled) {
        System.out.println("Angled.");
        m_shooter.spinAngleMotor(0); // Resets the angle motor to 0, ends command
    }
}
