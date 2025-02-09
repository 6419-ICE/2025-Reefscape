package frc.robot.commands;

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
    
}
