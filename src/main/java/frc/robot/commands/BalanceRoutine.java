package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivebaseSubsystem;


/** Autonomously drives over the charge station, then reverses, and balances itself on top.
 * <p><strong>SETUP:</strong> Place front of robot directly facing charge station. Be sure to be far enough away that it can get the
 * speed it needs,but not too far that it takes to long to get there.
 * <p><strong>END:</strong> The robot is balanced on top of the charge station
 * <p><strong>SCORES:</strong> Auto mobility, auto engaged
 */
public class BalanceRoutine extends SequentialCommandGroup {
    DrivebaseSubsystem m_drivebase;

    public BalanceRoutine(DrivebaseSubsystem drivebase) {
        super();
        m_drivebase = drivebase;

        this.addCommands(new DriveUntilTipped(drivebase, 8, 0.5));
        this.addCommands(new DriveUntilTipped(drivebase, -8, 0.5));
        this.addCommands(new DriveUntilTipped(drivebase, 0, 0.5));
        this.addCommands(new DriveDistance(drivebase, 0.5, 10));
        this.addCommands(new WaitCommand(0.5));
        this.addCommands(new DriveUntilTipped(drivebase, -8, -0.5));
        this.addCommands(new AutoBalance(drivebase));
    }
}
