package frc.robot.subsystems;

import java.net.http.HttpRequest;
import java.util.Arrays;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorPositions;

public class LEDSubsystem extends SubsystemBase {

    // Used for iterating through the unique elevator heights when rounding
    private static final double[] ELEVATOR_HEIGHTS = {
        ElevatorPositions.inside.getValue(),
        ElevatorPositions.L1.getValue(),
        ElevatorPositions.L2.getValue(),
        ElevatorPositions.L3.getValue(),
        ElevatorPositions.L4.getValue()
    };

    // The height at which the elevator LEDs should be all lit up (L4)
    private static final double ELEVATOR_FILLED = ELEVATOR_HEIGHTS[4];

    // Pattern used when the elevator isn't paused.
    // Blue LEDs follow the elevator's height up and down the strip.
    public static final LEDPattern ELEVATOR_STANDARD = 
        LEDPattern.solid(flipColor(Color.kBlue))
        .mask(LEDPattern.progressMaskLayer(LEDSubsystem::elevatorSupplier));

    // Pattern used when the elevator is paused/has a fault.
    // Bright Orange to let driver know there is a fault.
    public static final LEDPattern ELEVATOR_ERR = LEDPattern.solid(flipColor(Color.kOrange));

    // Object used to control the LED strip
    private AddressableLED leds;

    // Buffer storing the data to be sent to the LED strip
    private AddressableLEDBuffer buffer;

    // Views representing the 4 individual strips on the Elevator
    private AddressableLEDBufferView[] elevatorViews = new AddressableLEDBufferView[4];

    // Current pattern to display on the elevator
    private LEDPattern elevatorPattern = ELEVATOR_ERR;

    private static LEDSubsystem instance = null;
    private static DoubleSupplier elevatorPosSupplier = ()->0.0;
    

    public LEDSubsystem(DoubleSupplier elevatorPosSupplier) {
        leds = new AddressableLED(LEDConstants.LEDPort);
        buffer = new AddressableLEDBuffer(LEDConstants.LEDStripCount*4);
        leds.setLength(buffer.getLength());
        leds.setData(buffer);
        leds.start();
        elevatorViews[0] = buffer.createView(0, LEDConstants.LEDStripCount-1);
        elevatorViews[1] = buffer.createView(LEDConstants.LEDStripCount, LEDConstants.LEDStripCount*2-1).reversed();
        elevatorViews[2] = buffer.createView(LEDConstants.LEDStripCount*2, LEDConstants.LEDStripCount*3-1);
        elevatorViews[3] = buffer.createView(LEDConstants.LEDStripCount*3, Constants.LEDConstants.LEDStripCount*4-1).reversed();
        LEDSubsystem.elevatorPosSupplier = elevatorPosSupplier;
        instance = this;
    }
    @Override
    public void periodic() {
        for (AddressableLEDBufferView view : elevatorViews) {
            elevatorPattern.applyTo(view);
        }
        leds.setData(buffer);
    }
    
    public static void setElevatorPattern(LEDPattern pattern) {
        if (instance != null) instance.elevatorPattern = pattern;
    }
    
    private static double elevatorSupplier() {
        double height = elevatorPosSupplier.getAsDouble();
        if (height == -1) return ELEVATOR_HEIGHTS[4]; // invalid pos
        for (double elevatorPos : ELEVATOR_HEIGHTS) {
            // TODO elevator not going high enough
            if (MathUtil.isNear(elevatorPos, height, ElevatorConstants.tolerance*1.5)) {
                height = elevatorPos;
                break;
            }
        }
        
        return Math.max(height, 0.0) / ELEVATOR_HEIGHTS[4];
    }

    private static Color flipColor(Color color) {
        return new Color(color.green,color.red,color.blue);
    }
    private static Color[] flipColors(Color... colors) {
        return Arrays.stream(colors).map(LEDSubsystem::flipColor).toArray(Color[]::new);
    }
    
    public void stop() {
        leds.stop();
    }
}
