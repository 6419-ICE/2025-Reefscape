package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.subsystems.ElevatorSubsystem;
public class MoveElevatorCommand extends Command {
    private Supplier<ElevatorPositions> targetPosition;
    private ElevatorSubsystem elevator;
    public MoveElevatorCommand(ElevatorPositions targetPosition, ElevatorSubsystem elevator) {
        this(()->targetPosition,elevator);
    }
    public MoveElevatorCommand(Supplier<ElevatorPositions> targetPosition, ElevatorSubsystem elevator) {
        this.targetPosition = targetPosition;
        addRequirements(
            this.elevator = elevator
        );
    }
    @Override
    public void initialize() {
        elevator.setPosition(targetPosition.get().getValue());
    }
    @Override
    public void execute() {
        //elevator.setPosition(targetPosition.get().getValue());
    }
    
    @Override
    public void end(boolean interrupted) {
        //idk really know what to do on interrupt so I just do nothing :|
    }
    @Override
    public boolean isFinished() {
        return elevator.atPosition();
    }
    
}
