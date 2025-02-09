package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;
public class IntakeSubsystem extends SubsystemBase {
    private PowerDistribution pdh;
    private DigitalInput lineBreak;
    private SparkMax motor1,motor2;
    //TODO find out motor info
    public IntakeSubsystem() {
        pdh = new PowerDistribution();
        pdh.setSwitchableChannel(false);
        lineBreak = new DigitalInput(IntakeConstants.lineBreakPort);
        motor1 = new SparkMax(IntakeConstants.intakeMotorID1, MotorType.kBrushless);
        motor2 = new SparkMax(IntakeConstants.intakeMotorID2, MotorType.kBrushless);
    }
    public void dropIntake() {
        pdh.setSwitchableChannel(true);
    }
    public void setSpeed(double speed) {
        motor1.set(speed);
        motor2.set(speed);
    }
    public boolean hasCoral() {
        return lineBreak.get();
    }
}
