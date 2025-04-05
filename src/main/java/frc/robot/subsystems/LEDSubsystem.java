package frc.robot.subsystems;

import java.util.Arrays;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
    private AddressableLED leds;
    private AddressableLEDBufferView leftView,rightView;
    private AddressableLEDBuffer buffer;
    private LEDPattern pattern = LEDPattern.kOff;
        public LEDSubsystem() {
        leds = new AddressableLED(LEDConstants.LEDPort);
        buffer = new AddressableLEDBuffer(LEDConstants.LEDCount*2);
        leds.setLength(buffer.getLength());
        leds.start();
        leftView = buffer.createView(0, LEDConstants.LEDCount-1);
        rightView = buffer.createView(LEDConstants.LEDCount, LEDConstants.LEDCount*2-1);
    }
    @Override
    public void periodic() {
     //   pattern.applyTo(leftView);
      //  pattern.applyTo(rightView);
     //   leds.setData(buffer);
    }
    public void setPattern(LEDPattern pattern) {
        this.pattern = pattern;
    }
    public static Color flipColor(Color color) {
        return new Color(color.green,color.red,color.blue);
    }
    public static Color[] flipColors(Color... colors) {
        return Arrays.stream(colors).map(LEDSubsystem::flipColor).toArray(Color[]::new);
    }
    public static LEDPattern noCoralPattern() {
        return LEDPattern.solid(flipColor(Color.kRed));
    }
    public static LEDPattern coralPattern() {
        return LEDPattern.solid(flipColor(Color.kGreen));
    }
    public static LEDPattern intakingPattern() {
        return LEDPattern.gradient(GradientType.kContinuous, flipColors(Color.kBlack,Color.kYellow)).scrollAtRelativeSpeed(Units.Hertz.of(0.5));
    }
    public void stop() {
        leds.stop();
    }
}
