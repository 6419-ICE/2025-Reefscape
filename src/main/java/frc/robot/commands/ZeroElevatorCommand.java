package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.ElevatorSubsystem;

public class ZeroElevatorCommand extends RunCommand {
    private ElevatorSubsystem elevator;
    public ZeroElevatorCommand(ElevatorSubsystem elevator) {
        super(()->elevator.setPower(-0.05),elevator);
        this.elevator = elevator;
    }
    @Override
    public boolean isFinished() {
        return elevator.atZero();
    }
    @Override
    public void end(boolean interrupted) {
        elevator.setPower(0);
    }
}
