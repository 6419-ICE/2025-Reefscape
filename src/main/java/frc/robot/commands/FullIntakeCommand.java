package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorAndOuttakePositions;
import frc.robot.Constants.IntakeStates;
import frc.robot.Constants.OuttakeStates;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

public class FullIntakeCommand extends SequentialCommandGroup {
    public FullIntakeCommand(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem, OuttakeSubsystem outtakeSubsystem) {
        addCommands(
            new MoveElevatorAndOuttakeCommand(ElevatorAndOuttakePositions.intake, elevatorSubsystem, outtakeSubsystem),
            new RunIntakeCommand(IntakeStates.intake, intakeSubsystem).until(intakeSubsystem::hasCoral),
            new MoveElevatorAndOuttakeCommand(ElevatorAndOuttakePositions.inside, elevatorSubsystem, outtakeSubsystem),
            new RunOuttakeCommand(OuttakeStates.intake, outtakeSubsystem).until(outtakeSubsystem::hasCoral)
        );
    }
    
}
