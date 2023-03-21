package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


/** Autonomously drives over the charge station, then reverses, and balances itself on top.
 * <p><strong>SETUP:</strong> Place front of robot directly facing charge station. Be sure to be far enough away that it can get the
 * speed it needs,but not too far that it takes to long to get there.
 * <p><strong>END:</strong> The robot is balanced on top of the charge station
 * <p><strong>SCORES:</strong> Auto mobility, auto engaged
 */
public class LoadPieceSequence extends SequentialCommandGroup {
    DrivebaseSubsystem m_drivebase;
    ShooterSubsystem m_shooter;
    LedSubsystem m_led;

    public LoadPieceSequence(DrivebaseSubsystem drivebase, ShooterSubsystem shooter, LedSubsystem led) {
        m_drivebase = drivebase;
        m_shooter = shooter;
        m_led = led;

        // this.addCommands(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Yellow)));

        this.addCommands(new LoadMoveDistance(m_drivebase,.2));

        this.addCommands(new InstantCommand(() -> m_shooter.setFlap(DoubleSolenoid.Value.kForward)));
        this.addCommands(new InstantCommand(() -> m_shooter.setBucketCylinders(DoubleSolenoid.Value.kReverse, DoubleSolenoid.Value.kReverse)));

        this.addCommands(new BucketToZero(m_shooter, 0.5));
    }
}