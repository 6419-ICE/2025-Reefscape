package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FlipperAngles;
import frc.robot.Constants.FlipperStates;
import frc.robot.subsystems.FlipperSubsystem;

public class RunFlipperCommand extends Command {
    private FlipperSubsystem flipperSubsystem;
    public RunFlipperCommand(FlipperSubsystem flipperSubsystem) {
        this.flipperSubsystem = flipperSubsystem;
    }
    @Override
    public void initialize() {
        flipperSubsystem.setAngle(FlipperAngles.extended.getValue());
        flipperSubsystem.setWheelSpeed(FlipperStates.running.getValue());
    }
    @Override
    public void end(boolean interrupted) {
        flipperSubsystem.setAngle(FlipperAngles.inside.getValue());
        flipperSubsystem.setWheelSpeed(FlipperStates.idle.getValue());
    }
}
