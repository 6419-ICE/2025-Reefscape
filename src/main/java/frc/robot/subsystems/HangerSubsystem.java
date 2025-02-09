package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.HangerConstants;

public class HangerSubsystem {
    private TalonFX motor;
    public HangerSubsystem() {
        motor = new TalonFX(HangerConstants.motorID);
        
    }
    public void setAngle(double angle) {
        motor.setPosition(angle);
    }
    public boolean atAngle() {
        return motor.getVelocity().getValueAsDouble() <= 1;
    }
}
