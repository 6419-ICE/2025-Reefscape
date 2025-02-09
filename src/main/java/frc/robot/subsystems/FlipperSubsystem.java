package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.FlipperConstants;
public class FlipperSubsystem {
    private SparkMax rotationMotor, wheelMotor;
    private SparkClosedLoopController rotationController;
    public FlipperSubsystem() {
        rotationMotor = new SparkMax(FlipperConstants.rotationMotorID, MotorType.kBrushless);
        wheelMotor = new SparkMax(FlipperConstants.wheelMotorID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.absoluteEncoder.positionConversionFactor(360.0/8192.0);
        config.closedLoop.pid(FlipperConstants.rotationP,FlipperConstants.rotationI,FlipperConstants.rotationD);
        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        config.softLimit.forwardSoftLimit(90);
        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimit(0);
        config.softLimit.reverseSoftLimitEnabled(true);
        rotationMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        rotationController = rotationMotor.getClosedLoopController();
    }
    public void setAngle(double angle) {
        rotationController.setReference(angle, ControlType.kPosition);
    }
    public void setWheelSpeed(double speed) {
        wheelMotor.set(speed);
    }
    public double getAngle() {
        return rotationMotor.getAbsoluteEncoder().getPosition();
    }
    public double getWheelSpeed() {
        return wheelMotor.get();
    }
}
