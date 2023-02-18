package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivebaseSubsystem;

public class AutoBalance extends CommandBase {
    private DrivebaseSubsystem m_DrivebaseSubsystem;

    private double currentAngle;

    private final PIDController pid = new PIDController(Constants.Balance.DRIVE_KP, 0, 0.03);

    public AutoBalance(DrivebaseSubsystem subsystem) {
        m_DrivebaseSubsystem = subsystem;
        addRequirements(m_DrivebaseSubsystem);
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
        this.currentAngle = m_DrivebaseSubsystem.getRoll();

        double drivePower = -pid.calculate(currentAngle);
        System.out.println("Raw Drive Power " + drivePower);

        drivePower = MathUtil.clamp(drivePower, -0.2, 0.2);

        m_DrivebaseSubsystem.set(drivePower, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_DrivebaseSubsystem.set(0,0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }
}
