package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivebaseSubsystem;


/** Autonomously drives over the charge station, then reverses, and balances itself on top.
 * <p><strong>SETUP:</strong> Place front of robot directly facing charge station. Be sure to be far enough away that it can get the
 * speed it needs,but not too far that it takes to long to get there.
 * <p><strong>END:</strong> The robot is balanced on top of the charge station
 * <p><strong>SCORES:</strong> Auto mobility, auto engaged
 */
public class DriveForwardAutoRoutine extends SequentialCommandGroup {
    DrivebaseSubsystem m_drivebase;
    boolean m_timedOut = true;

    public DriveForwardAutoRoutine(DrivebaseSubsystem drivebase) {
        m_drivebase = drivebase;
        m_drivebase.setAutoOffset(90);

        var AutoFailedWidget = Shuffleboard.getTab("Auto").add("Auto status", false);
        AutoFailedWidget.withProperties(Map.of("colorWhenFalse", "grey"));
        // var colorWidget = Shuffleboard.getTab("Vision").add("Vision status", false);
        // colorWidget.withProperties(Map.of("colorWhenFalse", "grey"));
      
        this.addCommands(
          new FailFastTimeoutGroup()
            .thenWithTimeout(new DriveDistance(drivebase, 0.2, 100), 15)
        );
    }
}