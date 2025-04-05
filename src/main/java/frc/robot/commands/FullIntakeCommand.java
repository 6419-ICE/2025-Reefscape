package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorAndOuttakePositions;
import frc.robot.Constants.IntakeStates;
import frc.robot.Constants.OuttakeStates;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

public class FullIntakeCommand extends SequentialCommandGroup {
    public FullIntakeCommand(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem, OuttakeSubsystem outtakeSubsystem) {
        addCommands(
            //new MoveElevatorAndOuttakeCommand(ElevatorAndOuttakePositions.intake, elevatorSubsystem, outtakeSubsystem),
            // new RunIntakeCommand(IntakeStates.intake, intakeSubsystem).until(intakeSubsystem::hasCoral),
            new RotateOuttakeCommand(Constants.OuttakeAngles.intake,outtakeSubsystem),
            Commands.parallel(
                Commands.sequence(
                    //new MoveElevatorAndOuttakeCommand(ElevatorAndOuttakePositions.intake, elevatorSubsystem, outtakeSubsystem),
                    new ZeroElevatorCommand(elevatorSubsystem)
                ),
                Commands.race(
                    new RunOuttakeCommand(OuttakeStates.intake, outtakeSubsystem),
                    new RunIntakeCommand(IntakeStates.intake, intakeSubsystem),
                    new WaitUntilCommand(intakeSubsystem::hasCoral).andThen(new WaitCommand(1)).andThen(new WaitUntilCommand(()->!intakeSubsystem.hasCoral()))
                )
            )
        );
    }
    
}
