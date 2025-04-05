package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;
/**
 * class with static methods for logging to a corresponding program on the DS.
 */
public final class Logger {
    private static NetworkTableInstance inst;
    private static NetworkTable table;
    private static Map<String, GenericPublisher> map;

    public static void init() {
        inst = NetworkTableInstance.getDefault();
        inst.startServer();
        table = inst.getTable("datatable");
        map = new HashMap<>();
    }

    public static void logData(String key, String data) {
        if (!map.containsKey(key)) makeGeneric(key, NetworkTableType.kString);
        map.get(key).set(NetworkTableValue.makeString(data));
    }

    public static void logData(String key, double data) {
        if (!map.containsKey(key)) makeGeneric(key, NetworkTableType.kDouble);
        map.get(key).set(NetworkTableValue.makeDouble(data));
    }

    public static void logData(String key, boolean data) {
        if (!map.containsKey(key)) makeGeneric(key, NetworkTableType.kBoolean);
        map.get(key).set(NetworkTableValue.makeBoolean(data));
    }
    
    private static void makeGeneric(String key, NetworkTableType type) {
        map.put(key,table.getTopic(key).genericPublish(type.getValueStr()));
        
    }

    private Logger() {}

}
