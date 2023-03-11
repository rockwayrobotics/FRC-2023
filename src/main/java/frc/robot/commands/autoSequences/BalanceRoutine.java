package frc.robot.commands.autoSequences;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveUntilTipped;
import frc.robot.commands.FailFastTimeoutGroup;
import frc.robot.commands.RotateToAngle;
import frc.robot.subsystems.DrivebaseSubsystem;


/** Autonomously drives over the charge station, then reverses, and balances itself on top.
 * <p><strong>SETUP:</strong> Place front of robot directly facing charge station. Be sure to be far enough away that it can get the
 * speed it needs,but not too far that it takes to long to get there.
 * <p><strong>END:</strong> The robot is balanced on top of the charge station
 * <p><strong>SCORES:</strong> Auto mobility, auto engaged
 */
public class BalanceRoutine extends SequentialCommandGroup {
    DrivebaseSubsystem m_drivebase;

    public void setStatusWidget(SimpleWidget AutoFailedWidget, FailFastTimeoutGroup sequence) {
        if(sequence.timedOut()) {
            AutoFailedWidget.withProperties(Map.of("colorWhenFalse", "red"));
        } else {
            AutoFailedWidget.withProperties(Map.of("colorWhenFalse", "green"));
        }
    }

    public BalanceRoutine(DrivebaseSubsystem drivebase, SimpleWidget AutoFailedWidget) {
        m_drivebase = drivebase;
        m_drivebase.setAutoOffset(90);

        AutoFailedWidget.withProperties(Map.of("colorWhenFalse", "yellow"));

        FailFastTimeoutGroup sequence = new FailFastTimeoutGroup()
            .thenWithTimeout(new RotateToAngle(drivebase, 0, 3, .2), 5)
            .thenWithTimeout(new DriveUntilTipped(drivebase, -12, 0.4), 15)
            .thenWithTimeout(new DriveUntilTipped(drivebase, 12, 0.2), 15)
            .thenWithTimeout(new DriveUntilTipped(drivebase, 0, 0.2), 15)
            .thenWithTimeout(new DriveDistance(drivebase, 0.2, 30), 15)
            .then(new WaitCommand(0.5))
            .thenWithTimeout(new DriveUntilTipped(drivebase, 14, -0.4), 15)
            .then(new AutoBalance(drivebase))
            .then(new WaitCommand(0.5))
            .then(new AutoBalance(drivebase));


        this.addCommands(sequence);
        this.addCommands(new InstantCommand(() -> setStatusWidget(AutoFailedWidget, sequence)));
    }
}