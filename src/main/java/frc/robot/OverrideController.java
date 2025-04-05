package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.commands.ZeroElevatorCommand;
import frc.robot.subsystems.ElevatorSubsystem;

public class OverrideController extends CommandGenericHID {
    public OverrideController() {
        super(3);
    }
    public void enable(ElevatorSubsystem elevatorSubsystem) {
        button(1).or(button(2)).whileTrue(new ConditionalCommand(
            new FunctionalCommand(()->elevatorSubsystem.setPower(0.05), ()->{}, (bool)->elevatorSubsystem.setPower(0), ()->false, elevatorSubsystem), 
            new FunctionalCommand(()->elevatorSubsystem.setPower(-0.05), ()->{}, (bool)->elevatorSubsystem.setPower(0), ()->false, elevatorSubsystem), 
            ()->getHID().getRawButton(1)
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    }
}
