package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorStressTest extends SequentialCommandGroup {
    public ElevatorStressTest(ElevatorSubsystem elevatorSubsystem) {
        super(
            new MoveElevatorCommand(ElevatorPositions.L2, elevatorSubsystem),
            new MoveElevatorCommand(ElevatorPositions.L3, elevatorSubsystem),
            new MoveElevatorCommand(ElevatorPositions.inside, elevatorSubsystem),
            new ZeroElevatorCommand(elevatorSubsystem)
        );
    }
    
}
