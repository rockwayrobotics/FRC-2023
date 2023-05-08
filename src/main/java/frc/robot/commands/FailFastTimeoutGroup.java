package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * CommandGroup that stops running all remaining commands if any command times out.
 */
public class FailFastTimeoutGroup extends SequentialCommandGroup {
    boolean m_timedOut = false;

    public FailFastTimeoutGroup() {
        this.addCommands(new InstantCommand(() -> {m_timedOut = false;}));
    }

    /**
     * Adds a new command to the group with a timeout. Commands added after this will not run if this timeout is reached.
     * @param command Command to add to the group
     * @param timeout Timeout, in seconds
     * @return This timeout group, for chaining
     */
    public FailFastTimeoutGroup thenWithTimeout(Command command, double timeout) {
        this.addCommands(new ConditionalCommand(
            new InstantCommand(() -> {}),
            command.finallyDo(this::commandEnd).withTimeout(timeout),
            this::timedOut
        ));
        return this;
    }

    /**
     * Adds a new command to the group, with no timeout.
     * @param command Command to add to the group
     * @return This timeout group, for chaining
     */
    public FailFastTimeoutGroup then(Command command) {
        this.addCommands(command);
        return this;
    }

    /**
     * Adds a new command which will only run in a previous command in the group timed out. Useful to add an "error handler" command.
     * @param command Command to run if previous command times out
     * @return This timeout group, for chaining
     */
    public FailFastTimeoutGroup thenIfTimedOut(Command command) {
        this.addCommands(new ConditionalCommand(
            command,
            new InstantCommand(() -> {}),
            this::timedOut
        ));
        return this;
    }

    void commandEnd(boolean cancelled) {
        if (cancelled) {
            m_timedOut = true;
        }
    }

    /**
     * Gets whether any command in the group has timed out yet.
     * @return true if a timeout has been reached, false otherwise
     */
    public boolean timedOut() {
        return m_timedOut;
    }
}
