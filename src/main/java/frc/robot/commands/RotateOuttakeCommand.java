package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.Constants.OuttakeAngles;

public class RotateOuttakeCommand extends Command {
    private OuttakeSubsystem outtake;
    private Supplier<OuttakeAngles> targetAngle;
    public RotateOuttakeCommand(OuttakeAngles targetAngle, OuttakeSubsystem outtake) {
        this.targetAngle = ()->targetAngle;
        addRequirements(
            this.outtake = outtake
        );
    }
    public RotateOuttakeCommand(Supplier<OuttakeAngles> targetAngle, OuttakeSubsystem outtake) {
        this.targetAngle = targetAngle;
        addRequirements(
            this.outtake = outtake
        );
    }
    @Override
    public void initialize() {
        outtake.setAngle(targetAngle.get().getValue());
    }
    @Override
    public void execute() {
        outtake.setAngle(targetAngle.get().getValue());
    }
    @Override
    public void end(boolean interrupted) {
        //same situation as elevator, idk what to do here :|
    }
    @Override
    public boolean isFinished() {
        return outtake.atAngle();
    }
}
