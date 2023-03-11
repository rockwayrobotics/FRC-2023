package frc.robot.commands.autoSequences;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveUntilTipped;
import frc.robot.commands.FailFastTimeoutGroup;
import frc.robot.subsystems.DrivebaseSubsystem;


/** Autonomously drives over the charge station, and stops
 * <p><strong>SETUP:</strong> Place front of robot directly facing charge station. Be sure to be far enough away that it can get the
 * speed it needs,but not too far that it takes to long to get there.
 * <p><strong>END:</strong> The robot is sitting on the opposite side of the charge station outside the community
 * <p><strong>SCORES:</strong> Auto mobility
 */
public class NoTurnCommunityRoutine extends SequentialCommandGroup {
    DrivebaseSubsystem m_drivebase;
    boolean m_timedOut = true;

    SimpleWidget AutoFailedWidget = Shuffleboard.getTab("Auto").add("Auto status", false);


    FailFastTimeoutGroup sequence;

    public void DetermineRoutineFailure() {
        if(sequence.timedOut()) {
            AutoFailedWidget.withProperties(Map.of("colorWhenFalse", "red"));
        }
    }

    public NoTurnCommunityRoutine(DrivebaseSubsystem drivebase) {
        m_drivebase = drivebase;
        m_drivebase.setAutoOffset(90);

        sequence = new FailFastTimeoutGroup()
                .thenWithTimeout(new DriveUntilTipped(drivebase, -12, 0.4), 15)
                .thenWithTimeout(new DriveUntilTipped(drivebase, 12, 0.2), 15)
                .thenWithTimeout(new DriveUntilTipped(drivebase, 0, 0.2), 15)
                .thenWithTimeout(new DriveDistance(drivebase, 0.2, 30), 15);

        AutoFailedWidget.withProperties(Map.of("colorWhenFalse", "grey"));

        this.addCommands(sequence);
        this.addCommands(new InstantCommand());
    }
}