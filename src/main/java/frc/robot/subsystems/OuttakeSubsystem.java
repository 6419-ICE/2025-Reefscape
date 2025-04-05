package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.util.sendable.AnnotatedSendable;

public class OuttakeSubsystem extends SubsystemBase implements AnnotatedSendable {
    private SparkMax outtakeMotor, rotationMotor;
    private SparkClosedLoopController rotationController;

    @Variable(key="Target Angle", mutable=false)
    private double targetAngle = 0.0;

    private DigitalInput lineBreak;

    public OuttakeSubsystem() {
        outtakeMotor = new SparkMax(OuttakeConstants.outtakeMotorID, MotorType.kBrushless);
        rotationMotor = new SparkMax(OuttakeConstants.rotationMotorID, MotorType.kBrushless);
        SparkMaxConfig rotationConfig = new SparkMaxConfig();
        //convert from native resolution (8192 per rev) to degrees
        // rotationConfig.absoluteEncoder.positionConversionFactor(360.0/8192.0);
        rotationConfig.closedLoop.pid(OuttakeConstants.rotationP, OuttakeConstants.rotationI, OuttakeConstants.rotationD);
        rotationConfig.idleMode(IdleMode.kBrake);
        rotationConfig.closedLoop.maxOutput(1);
        rotationConfig.softLimit.forwardSoftLimitEnabled(false);
        rotationConfig.softLimit.reverseSoftLimitEnabled(false);
        rotationConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        rotationMotor.configure(rotationConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        rotationController = rotationMotor.getClosedLoopController();
        rotationMotor.getEncoder().setPosition(0);
        SparkMaxConfig outtakeConfig = new SparkMaxConfig();
        outtakeConfig.smartCurrentLimit(15);
        lineBreak = new DigitalInput(OuttakeConstants.lineBreakPort);
    }
    public void setOuttake(double power) {
        outtakeMotor.set(power);
    }
    public void setAngle(double angle) {
        rotationController.setReference(((targetAngle = angle)/360) * 60.0 * (48.0/18.0), ControlType.kPosition);
    }
    public void setRotationPower(double power) {
        rotationController.setReference(power, ControlType.kDutyCycle);
    }

    //@Getter(key="Angle")
    public double getAngle() {
        return ((360*rotationMotor.getEncoder().getPosition())/60.0) * (18.0/48.0);
    }
    public double getOuttake() {
        return outtakeMotor.get();
    }

    @Getter(key="At Position")
    public boolean atAngle() {
        return Math.abs(getAngle()-targetAngle) <= OuttakeConstants.tolerance;
    }
    public boolean hasCoral() {
        return lineBreak.get();
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        AnnotatedSendable.super.initSendable(builder);
        builder.addDoubleProperty("Rotation Power Output", rotationMotor::getAppliedOutput, null);
        builder.addDoubleProperty("Angle", this::getAngle, null);
    }
}