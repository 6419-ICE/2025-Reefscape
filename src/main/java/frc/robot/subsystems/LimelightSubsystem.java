package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.sendable.AnnotatedSendable;

public class LimelightSubsystem implements Subsystem, AnnotatedSendable {
    public enum Side {
        left,
        right;
    }
    private static final String id = "limelight-";
    private final NetworkTable limelight;
    private final NetworkTableEntry tx, ty, tv, targetpose_robotspace;
    private final Side side;
    public LimelightSubsystem(Side side) {
        this.side = side;
        limelight = NetworkTableInstance.getDefault().getTable(id + side.toString());
        //x offset from the target in degrees
        tx = limelight.getEntry("tx");
        //y offset from the target in degrees
        ty = limelight.getEntry("ty");
        //"boolean" value that states if there is a visible target
        tv = limelight.getEntry("tv");
        //3D transform of the primary in-view AprilTag in the coordinate system of the Camera (array (6)) [tx, ty, tz, pitch, yaw, roll] (meters, degrees)
        targetpose_robotspace = limelight.getEntry("targetpose_robotspace");
    }

    @Getter(key="Sees Target")
    public boolean hasVisibleTarget() {
        return tv.getInteger(0) == 1;
    }
    public double getXOffset() {
        return tx.getDouble(0.0);
    }
    public double getYOffset() {
        return ty.getDouble(0.0);
    }
    public double[] getTargetPoseCameraSpace() {
        return targetpose_robotspace.getDoubleArray(new double[0]);
    }
    @Getter(key="X Offset")
    public double get3dXOffset() {
        return targetpose_robotspace.getDoubleArray(new double[6])[0];
    }
    @Getter(key="Y Offset")
    public double get3dYOffset() {
        return targetpose_robotspace.getDoubleArray(new double[6])[1];
    }
    @Getter(key="Z Offset")
    public double get3dZOffset() {
        return targetpose_robotspace.getDoubleArray(new double[6])[2];
    }
    public double get3dPitchOffset() {
        return targetpose_robotspace.getDoubleArray(new double[6])[3];
    }
    @Getter(key="Yaw Offset")
    public double get3dYawOffset() {
        return targetpose_robotspace.getDoubleArray(new double[6])[4];
    }
    public double get3dRollOffset() {
        return targetpose_robotspace.getDoubleArray(new double[6])[5];
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Vertical Offset", getYOffset());
        SmartDashboard.updateValues();
    }
    public Side getSide() {
        return side;
    }
    
}
