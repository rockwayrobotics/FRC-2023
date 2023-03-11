package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


/** Autonomously drives over the charge station, then reverses, and balances itself on top.
 * <p><strong>SETUP:</strong> Place front of robot directly facing charge station. Be sure to be far enough away that it can get the
 * speed it needs,but not too far that it takes to long to get there.
 * <p><strong>END:</strong> The robot is balanced on top of the charge station
 * <p><strong>SCORES:</strong> Auto mobility, auto engaged
 */
public class ShootSequence extends SequentialCommandGroup {
    DrivebaseSubsystem m_drivebase;
    ShooterSubsystem m_shooter;
    LedSubsystem m_led;

    public ShootSequence(DrivebaseSubsystem drivebase, ShooterSubsystem shooter, LedSubsystem led) {
        m_drivebase = drivebase;
        m_shooter = shooter;
        m_led = led;

        this.addCommands(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Yellow)));

        // Take over drivebase
        switch (m_shooter.m_scoringTarget) {
            case HIGH_CUBE -> this.addCommands(new DriveDistance(m_drivebase,.2, Units.inchesToMeters(m_shooter.highCubeBackupDistanceInches)));
            case MID_CUBE -> this.addCommands(new DriveDistance(m_drivebase,.2, Units.inchesToMeters(m_shooter.midCubeBackupDistanceInches)));
            case MID_CONE -> this.addCommands(new DriveDistance(m_drivebase,.2, Units.inchesToMeters(m_shooter.midConeBackupDistanceInches)));
        };

        this.addCommands(new InstantCommand(() -> m_shooter.setFlap(DoubleSolenoid.Value.kForward)));
        this.addCommands(new InstantCommand(() -> m_shooter.setBucketCylinders(DoubleSolenoid.Value.kForward, DoubleSolenoid.Value.kForward)));
        // Start driving here
        this.addCommands(new WaitCommand(.5));
        this.addCommands(new InstantCommand(() -> m_shooter.setBucketCylinders(DoubleSolenoid.Value.kReverse, DoubleSolenoid.Value.kReverse)));
        this.addCommands(new WaitCommand(1));
        this.addCommands(new InstantCommand(() -> m_shooter.setFlap(DoubleSolenoid.Value.kReverse)));
    }
}