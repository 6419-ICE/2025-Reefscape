package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuttakeConstants;

public class OuttakeSubsystem extends SubsystemBase {
    private SparkMax outtakeMotor, rotationMotor;
    private SparkClosedLoopController rotationController;
    private double targetAngle = 0.0;
    private DigitalInput lineBreak;
    public OuttakeSubsystem() {
        outtakeMotor = new SparkMax(OuttakeConstants.outtakeMotorID, MotorType.kBrushless);
        rotationMotor = new SparkMax(OuttakeConstants.rotationMotorID, MotorType.kBrushless);
        SparkMaxConfig rotationConfig = new SparkMaxConfig();
        //convert from native resolution (8192 per rev) to degrees
        rotationConfig.absoluteEncoder.positionConversionFactor(360.0/8192.0);
        rotationConfig.closedLoop.pid(OuttakeConstants.rotationP, OuttakeConstants.rotationI, OuttakeConstants.rotationD);
        rotationConfig.softLimit.forwardSoftLimit(180);
        rotationConfig.softLimit.forwardSoftLimitEnabled(true);
        rotationConfig.softLimit.reverseSoftLimitEnabled(true);
        rotationConfig.softLimit.reverseSoftLimit(0);
        rotationConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        rotationMotor.configure(rotationConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        rotationController = rotationMotor.getClosedLoopController();
        
        lineBreak = new DigitalInput(OuttakeConstants.lineBreakPort);
    }
    public void setOuttake(double power) {
        outtakeMotor.set(power);
    }
    public void setAngle(double angle) {
        rotationController.setReference(targetAngle = angle, ControlType.kPosition);
    }
    public double getAngle() {
        return rotationMotor.getAbsoluteEncoder().getPosition();
    }
    public double getOuttake() {
        return outtakeMotor.get();
    }
    public boolean atAngle() {
        return Math.abs(getAngle()-targetAngle) <= OuttakeConstants.tolerance;
    }
    public boolean hasCoral() {
        return lineBreak.get();
    }
}