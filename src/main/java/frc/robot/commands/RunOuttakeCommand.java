package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OuttakeStates;
import frc.robot.subsystems.OuttakeSubsystem;

public class RunOuttakeCommand extends Command {
    private OuttakeStates state;
    private OuttakeSubsystem outtakeSubsystem;
    public RunOuttakeCommand(OuttakeStates state, OuttakeSubsystem outtakeSubsystem) {
        this.state = state;
        addRequirements(
            this.outtakeSubsystem = outtakeSubsystem
        );
    }
    @Override
    public void initialize() {
        outtakeSubsystem.setOuttake(state.getValue());
    }
    @Override
    public void end(boolean interrupted) {
        outtakeSubsystem.setOuttake(0);
    }
}
