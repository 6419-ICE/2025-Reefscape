package frc.robot.util.sendable;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.function.FloatConsumer;
import edu.wpi.first.util.function.FloatSupplier;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Arrays;
import java.util.function.*;

/**
 * Class for storing information about a property that can be added to a Sendable
 */
class Property {
    private Callable getter, setter;
    private Class<?> type;
    public Property(Callable getter, Callable setter, Class<?> type) {
        this.getter = getter;
        this.setter = setter;
        this.type = type;
    }
    public Property(Callable getter, Class<?> type) {
        this(getter,null,type);
    }
    public Callable getGetter() {
        return getter;
    }
    public Callable getSetter() {
        return setter;
    }
    public void setSetter(Callable setter) {
        this.setter = setter;
    }
    public Class<?> getType() {
        return type;
    }
    public void addToBuilder(SendableBuilder builder,String key) {
        //AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
        if (type.equals(int.class) || type.equals(long.class)) {
            builder.addIntegerProperty(
                    key,
                    ()->(long) getter.call(),
                    setter == null ? null : setter::call
            );
        } else if (type.equals(double.class)) {
            builder.addDoubleProperty(
                    key,
                    ()->(double) getter.call(),
                    setter == null ? null : setter::call
            );
        } else if (type.equals(float.class)) {
            builder.addFloatProperty(
                    key,
                    ()->(float) getter.call(),
                    setter == null ? null : setter::call
            );
        } else if (type.equals(boolean.class)) {
            builder.addBooleanProperty(
                    key,
                    ()->(boolean) getter.call(),
                    setter == null ? null : setter::call
            );
        } else if (type.equals(String.class)) {
            builder.addStringProperty(
                    key,
                    ()->(String) getter.call(),
                    setter == null ? null : setter::call
            );
        } else if (type.equals(int[].class)) {
            //print trace to make it more noticeable
            DriverStation.reportWarning("Adding an int[] property through annotations requires converting and is therefore more expensive, consider using long[] instead.",true);
            builder.addIntegerArrayProperty(
                    key,
                    //NOTE: This probably makes properties using int arrays MUCH more expensive, hence the warning I print to log ^
                    ()->Arrays.stream((int[])getter.call()).asLongStream().toArray(),
                    setter == null ? null : setter::call
            );
        } else if (type.equals(long[].class)) {
            builder.addIntegerArrayProperty(
                    key,
                    ()->(long[]) getter.call(),
                    setter == null ? null : setter::call
            );
        } else if (type.equals(double[].class)) {
            builder.addDoubleArrayProperty(
                    key,
                    ()->(double[]) getter.call(),
                    setter == null ? null : setter::call
            );
        } else if (type.equals(float[].class)) {
            builder.addFloatArrayProperty(
                    key,
                    ()->(float[]) getter.call(),
                    setter == null ? null : setter::call
            );
        } else if (type.equals(boolean[].class)) {
            builder.addBooleanArrayProperty(
                    key,
                    ()->(boolean[]) getter.call(),
                    setter == null ? null : setter::call
            );
        } else if (type.equals(String[].class)) {
            builder.addStringArrayProperty(
                    key,
                    ()->(String[]) getter.call(),
                    setter == null ? null : setter::call
            );
        } else if (type.isAssignableFrom(AutoCloseable.class)) {
            builder.addCloseable((AutoCloseable) getter.call());
        } else {
            throw new UnsupportedOperationException("A field or method annotated for sending has an invalid type (" + type.getSimpleName() + ")," +
                    " valid types are: int,long,float,double,boolean,String,int[],long[],float[],double[],boolean[],String[], and AutoCloseable");
        }
    }

    /**
     * Class to ease interfacing with both Fields and Methods.
     */
    static class Callable {
        private Function<Object[],Object> func;
        public Callable(Field field, Object parent, boolean getter) {
            func = getter ? (params) -> {
                try {
                    return field.get(parent);
                } catch (IllegalAccessException e) {
                    throw new RuntimeException(e);
                }
            } : (param) -> {
                try {
                    field.set(parent, param);
                } catch (IllegalAccessException e) {
                    throw new RuntimeException(e);
                }
                return null;
            };
        }
        public Callable(Method m, Object parent) {
            func = (params)-> {
                try {
                    return m.invoke(parent,params);
                } catch (IllegalAccessException | InvocationTargetException e) {
                    throw new RuntimeException(e);
                }
            };
        }
        public Object call(Object... param) {
            return func.apply(param);
        }
    }
}