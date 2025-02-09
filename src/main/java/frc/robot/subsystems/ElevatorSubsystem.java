package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private TalonFX leftMotor, rightMotor;
    private Encoder encoder;
    private PhoenixPIDController controller;
    private double goal = 0.0;
    private Timer timer = new Timer();
    public ElevatorSubsystem() {
        leftMotor = new TalonFX(ElevatorConstants.leftMotorID);
        rightMotor = new TalonFX(ElevatorConstants.rightMotorID);
        controller = new PhoenixPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
        controller.setTolerance(ElevatorConstants.tolerance);
        encoder = new Encoder(ElevatorConstants.encoderPortA, ElevatorConstants.encoderPortB);
        encoder.setDistancePerPulse(0.0); //TODO
    }
    @Override
    public void periodic() {
        leftMotor.set(controller.calculate(encoder.getDistance(), goal, timer.get()));
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), false));
    }
    public void setPosition(double pos) {
        goal = pos;
        timer.restart();
    }
    public double getPosition() {
        return encoder.getDistance();
    }
    public boolean atPosition() {
        return Math.abs(goal-getPosition()) <= ElevatorConstants.tolerance;
    }
}
