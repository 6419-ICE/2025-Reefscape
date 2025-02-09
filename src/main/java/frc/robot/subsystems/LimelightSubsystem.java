package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {
    private static final String id = "limelight";
    private final NetworkTable limelight;
    private final NetworkTableEntry tx, ty, tv, targetpose_cameraspace;
    public LimelightSubsystem() {
        limelight = NetworkTableInstance.getDefault().getTable(id);
        //x offset from the target in degrees
        tx = limelight.getEntry("tx");
        //y offset from the target in degrees
        ty = limelight.getEntry("ty");
        //"boolean" value that states if there is a visible target
        tv = limelight.getEntry("tv");
        //3D transform of the primary in-view AprilTag in the coordinate system of the Camera (array (6)) [tx, ty, tz, pitch, yaw, roll] (meters, degrees)
        targetpose_cameraspace = limelight.getEntry("targetpose_cameraspace");
    }
    public boolean hasVisibleTarget() {
        return tv.getInteger(0) == 1;
    }
    public double getXOffset() {
        return tx.getDouble(0.0);
    }
    public double getYOffset() {
        return ty.getDouble(0.0);
    }
    /**
     * get the *approximate* distance from the target in inches
     * taken from here: https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance 
     */
    public double getDistanceFromNote() {
        double verticalOffset = getYOffset();
        //height of the center of a note from the floor
        double goalHeight = 50.5;
        double goalAngleDegrees = Constants.LimelightConstants.kLimelightAngle + verticalOffset;
        double goalAngleRadians = Units.degreesToRadians(goalAngleDegrees);

        return (goalHeight-Constants.LimelightConstants.kLimelightHeightInches)/ Math.tan(goalAngleRadians);
    }
    public double get3dXOffset() {
        return Units.metersToInches(targetpose_cameraspace.getDoubleArray(new double[6])[0]);
    }
    public double get3dYOffset() {
        return Units.metersToInches(targetpose_cameraspace.getDoubleArray(new double[6])[1]);
    }
    public double get3dZOffset() {
        return Units.metersToInches(targetpose_cameraspace.getDoubleArray(new double[6])[2]);
    }
    public double get3dPitchOffset() {
        return targetpose_cameraspace.getDoubleArray(new double[6])[3];
    }
    public double get3dYawOffset() {
        return targetpose_cameraspace.getDoubleArray(new double[6])[4];
    }
    public double get3dRollOffset() {
        return targetpose_cameraspace.getDoubleArray(new double[6])[5];
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Vertical Offset", getYOffset());
        SmartDashboard.putNumber("Distance", getDistanceFromNote());
        SmartDashboard.updateValues();
    }
    
}
