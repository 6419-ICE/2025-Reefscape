package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.Side;

public class OrientToAprilTagCommand extends Command {
    private LimelightSubsystem limelightSubsystem;
    private DriveSubsystem driveSubsystem;
    private double xDiff,zDiff,angle;
    public OrientToAprilTagCommand(double xDiff, double zDiff, double angle, LimelightSubsystem limelightSubsystem, DriveSubsystem driveSubsystem) {
        addRequirements(
            this.limelightSubsystem = limelightSubsystem,
            this.driveSubsystem = driveSubsystem
        );
        this.xDiff = xDiff;
        this.zDiff = zDiff;
        this.angle = angle;
    }
    public OrientToAprilTagCommand(int level, boolean left, LimelightSubsystem limelightSubsystem, DriveSubsystem driveSubsystem) {
        this(level == 1 ? 0 : LimelightConstants.reefXDiff * (left ? -1 : 1), LimelightConstants.reefZDiff, 0, limelightSubsystem, driveSubsystem);
    }
    public OrientToAprilTagCommand(LimelightSubsystem limelightSubsystem, DriveSubsystem driveSubsystem) {
        this(
            limelightSubsystem.getSide() == Side.left ? LimelightConstants.reefXDiff : -LimelightConstants.reefXDiff,
            LimelightConstants.reefZDiff,
            0,
            limelightSubsystem,
            driveSubsystem
        );
    }
    @Override
    public void execute() {
        if (!limelightSubsystem.hasVisibleTarget()) {
            driveSubsystem.drive(0, 0, 0, false);
        } else {
            driveSubsystem.drive(
                -LimelightConstants.zTranslationController.calculate(limelightSubsystem.get3dZOffset(),zDiff), 
                LimelightConstants.xTranslationController.calculate(limelightSubsystem.get3dXOffset(),xDiff), 
                LimelightConstants.rotationController.calculate(limelightSubsystem.get3dYawOffset(),angle), 
                false
            );
        }
    }
    @Override
    public boolean isFinished() {
        return MathUtil.isNear(limelightSubsystem.get3dXOffset(), xDiff, LimelightConstants.reefXTolerance)
            && MathUtil.isNear(limelightSubsystem.get3dZOffset(), zDiff, LimelightConstants.reefZTolerance)
            && MathUtil.isNear(limelightSubsystem.get3dYawOffset(), angle, LimelightConstants.reefHeadingTolerance);
    }
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.driveRaw(0, 0, 0, false);
    }
}
