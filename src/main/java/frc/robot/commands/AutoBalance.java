package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivebaseSubsystem;

public class AutoBalance extends CommandBase {
    private DrivebaseSubsystem m_DrivebaseSubsystem;

    private double error;
    private double currentAngle;
    private double drivePower;

    public AutoBalance(DrivebaseSubsystem subsystem) {
        m_DrivebaseSubsystem = subsystem;
        addRequirements(m_DrivebaseSubsystem);
    }

    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Uncomment the line below this to simulate the gyroscope axis with a controller joystick
        // Double currentAngle = -1 * Robot.controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS) * 45;
        this.currentAngle = m_DrivebaseSubsystem.getRoll();

        error = Constants.Balance.GOAL_DEGREES - currentAngle;
        drivePower = -Math.min(Constants.Balance.DRIVE_KP * error, 1);

        // Our robot needed an extra push to drive up in reverse, probably due to weight imbalances
        if (drivePower < 0) {
        drivePower *= Constants.Balance.BACKWARDS_POWER_MULTIPLIER;
        }

        // Limit the max power
        if (Math.abs(drivePower) > 0.4) {
        drivePower = Math.copySign(0.4, drivePower);
        }

        m_DrivebaseSubsystem.set(drivePower, 0);
        
        // Debugging Print Statments
        System.out.println("Current Angle: " + currentAngle);
        System.out.println("Error " + error);
        System.out.println("Drive Power: " + drivePower);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_DrivebaseSubsystem.set(0,0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(error) < Constants.Balance.TOLERANCE_DEGREES; // End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees)
    }
}
