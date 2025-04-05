package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ElevatorAndOuttakePositions;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

public class MoveElevatorAndOuttakeCommand extends ParallelCommandGroup {
    public MoveElevatorAndOuttakeCommand(ElevatorAndOuttakePositions pos,ElevatorSubsystem elevator, OuttakeSubsystem outtake) {
        addCommands(
            new MoveElevatorCommand(pos.pos, elevator),
            new RotateOuttakeCommand(pos.angle, outtake)
        );
    }
    public MoveElevatorAndOuttakeCommand(Supplier<ElevatorAndOuttakePositions> pos,ElevatorSubsystem elevator, OuttakeSubsystem outtake) {
        addCommands(
            new MoveElevatorCommand(()->pos.get().pos, elevator),
            new RotateOuttakeCommand(()->pos.get().angle, outtake)
        );
    }
    
}
