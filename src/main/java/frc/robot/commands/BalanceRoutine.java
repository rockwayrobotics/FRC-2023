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
    boolean m_timedOut = true;

    public BalanceRoutine(DrivebaseSubsystem drivebase) {
        m_drivebase = drivebase;
        m_drivebase.setAutoOffset(-90);
      
        this.addCommands(
          new FailFastTimeoutGroup()
            .thenWithTimeout(new RotateToAngle(drivebase, 0, 3, .2), 15)
            .thenWithTimeout(new DriveUntilTipped(drivebase, -8, 0.2), 3)
            .thenWithTimeout(new DriveUntilTipped(drivebase, 8, 0.2), 5)
            .thenWithTimeout(new DriveUntilTipped(drivebase, 0, 0.2), 5)
            .thenWithTimeout(new DriveDistance(drivebase, 0.2, 20), 5)
            .then(new WaitCommand(0.5))
            .thenWithTimeout(new DriveUntilTipped(drivebase, 8, -0.2), 5)
            .then(new AutoBalance(drivebase))
        );
    }
}