package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivebaseSubsystem;

public class AutoBalance extends CommandBase {
    private final DrivebaseSubsystem m_DrivebaseSubsystem;

    private final PIDController pid;

    public AutoBalance(DrivebaseSubsystem subsystem) {
        m_DrivebaseSubsystem = subsystem;
        addRequirements(m_DrivebaseSubsystem);

        pid = new PIDController(m_DrivebaseSubsystem.balance_kP, 0, m_DrivebaseSubsystem.balance_kD);

        pid.setSetpoint(Constants.Balance.GOAL_DEGREES);
        pid.setTolerance(Constants.Balance.TOLERANCE_DEGREES);
    }

    @Override
    public void initialize() {
        System.out.println("Init balance");
        pid.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Uncomment the line below this to simulate the gyroscope axis with a controller joystick
        // Double currentAngle = -1 * Robot.controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS) * 45;
        double currentAngle = m_DrivebaseSubsystem.getRoll();

        double drivePower = pid.calculate(currentAngle);
        SmartDashboard.putNumber("Raw Drive Power (Auto Balance)", drivePower);

        drivePower = MathUtil.clamp(drivePower, -0.2, 0.2);

        m_DrivebaseSubsystem.set(drivePower, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_DrivebaseSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }
}
