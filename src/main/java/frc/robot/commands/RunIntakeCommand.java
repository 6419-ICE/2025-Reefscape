package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.IntakeStates;
import frc.robot.subsystems.IntakeSubsystem;;

public class RunIntakeCommand extends Command {
    private IntakeStates state;
    private IntakeSubsystem intakeSubsystem;

    public RunIntakeCommand(IntakeStates state, IntakeSubsystem intakeSubsystem) {
        this.state = state;
        addRequirements(
            this.intakeSubsystem = intakeSubsystem
        );
    }
    @Override
    public void initialize() {
        intakeSubsystem.setSpeed(state.getValue());
    }
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setSpeed(0);
    }
}
