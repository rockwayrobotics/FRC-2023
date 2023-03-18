package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ScoringMode;
import frc.robot.Constants.ScoringTarget;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAngle extends CommandBase {

    private final ShooterSubsystem m_shooter;
    private final DrivebaseSubsystem m_drivebase;
    private double m_maxSpeed;
    private double m_distance;
    // private ScoringTarget targetAngle;
    // private ScoringTarget m_angle;

    private final PIDController pid;

    public ShootAngle(DrivebaseSubsystem drivebase, ShooterSubsystem shooter, double maxSpeed) {
        m_shooter = shooter;
        m_drivebase = drivebase;
        m_maxSpeed = maxSpeed;
    
        addRequirements(m_shooter);

        pid = new PIDController(.02, 0, 0);
        pid.setTolerance(10);
    }

    @Override
    public void initialize() {
        // Resets encoder values to default
        m_drivebase.resetEncoders();

        switch (m_shooter.m_ScoringMode) {
            case CUBE -> m_distance = m_shooter.cubeAngleSetpoint;
            case CONE -> m_distance = m_shooter.coneAngleSetpoint;
            case FLAT -> m_distance = 0;
        };   

        pid.setSetpoint(m_distance);
        pid.reset();
    }

    @Override
    public void execute() {
        double currentAngle = m_shooter.getAngleEncoder();

        double spinPower = pid.calculate(currentAngle);

        spinPower = MathUtil.clamp(spinPower, -m_maxSpeed, m_maxSpeed);

        m_shooter.spinAngleMotor(spinPower);
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }

    @Override
    public void end(boolean cancelled) {
        System.out.println("Angled to: " + m_shooter.m_ScoringMode);
        m_shooter.spinAngleMotor(0); // Resets the angle motor to 0, ends command
    }
}
