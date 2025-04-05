package frc.robot.util.sendable;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.*;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Stream;

/**
 * <p>
 * {@link Sendable} sub-interface that allows for {@link Sendable#initSendable(SendableBuilder) building a Sendable} using Java Annotations.
 * This class supports all of the same types as Sendable, excluding {@link SendableBuilder#addRawProperty(String, String, Supplier, Consumer) raw properties} (int, long, float, double, boolean, String, int[], long[], float[], double[], boolean[], String[], and AutoCloseable).
 * Implementing classes can add a "Property" using the {@link Getter Getter}, {@link Setter Setter} (Methods), and {@link Variable Variable} (Fields) annotations.
 * </p>
 * Methods with the Getter Annotation should return a valid type, and have no parameters:
 * <pre>
 *{@code
 * @Getter(key="someKey")
 * public ValidType someGetter() {
 *     return someValue;
 * }
 *}
 *</pre>
 * Methods with the Setter Annotation should have one parameter of a valid type, and may return a value, though it will be ignored.
 * NOTE: Setters must have a corresponding getter with the same {@link Setter#key() key}:
 * <pre>
 *{@code
 * @Setter(key="someKey")
 * public void someSetter(ValidType value) {
 *     someValue = value;
 * }
 *}
 *</pre>
 * Fields with the Variable Annotation that are {@code final} will be added to the builder without a "setter", making them immutable on the dashboard.
 * To make a non-final field immutable, the optional {@link Variable#mutable() mutable} value can be set to false (the mutable value is ignored if the field is final):
 * <pre>
 *{@code
 * @Variable(key="someKey",mutable=false)
 * ValidType someValue = foo;
 *
 * @Variable(key="someOtherKey")
 * ValidType someOtherValue = bar;
 *}
 *</pre>
 * @see Getter
 * @see Setter
 * @see Variable
 */
public interface AnnotatedSendable extends Sendable {
    /**
     * Annotation used to mark "getter" methods for a property of the given key. For more details, see {@link AnnotatedSendable}.
     */
    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.METHOD)
    @interface Getter {
        String key();
    }
    /**
     * Annotation used to mark "setter" methods for a property of the given key. For more details, see {@link AnnotatedSendable}.
     */
    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.METHOD)
    @interface Setter {
        String key();
    }
    /**
     * Annotation used to mark fields for a property of the given key. For more details, see {@link AnnotatedSendable}.
     */
    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.FIELD)
    @interface Variable {
        String key();
        boolean mutable() default true;
    }
    @Override
    default void initSendable(SendableBuilder builder) {
        //value = Triple<Getter, Setter, ReturnType>
        Map<String, Property> map = new HashMap<>();

        //-------------------METHOD MAPPING-------------------
        Method[] methods = getClass().getMethods();
        //stream silliness
        Method[] getters = Arrays.stream(methods)
                .filter((m)->m.isAnnotationPresent(Getter.class))
                .filter((m)->m.getParameterCount()==0)
                .filter((m)->m.getReturnType()!=Void.class)
                .toArray(Method[]::new);
        Method[] setters = Arrays.stream(methods)
                .filter((m)->m.isAnnotationPresent(Setter.class))
                .filter((m)->m.getParameterCount()==1)
                .toArray(Method[]::new);
        //verify and map all getters and setters
        for (Method getter : getters) {
            String key = getter.getAnnotation(Getter.class).key();
            if (map.containsKey(key)) throw new RuntimeException("Cannot have multiple getters of the same value");
            map.put(key,new Property(new Property.Callable(getter,this),getter.getReturnType()));
        }
        for (Method setter : setters) {
            String key = setter.getAnnotation(Setter.class).key();
            if (!map.containsKey(key)) throw new RuntimeException("All keys must have corresponding getters");
            if (!map.get(key).getType().equals(setter.getParameterTypes()[0])) throw new RuntimeException("Keys cannot have getters and setters for different value types");
            map.get(key).setSetter(new Property.Callable(setter,this));
        }
        //-------------------FIELD MAPPING-------------------
        Field[] fields = Arrays.stream(getClass().getDeclaredFields())
                .filter((f)->f.isAnnotationPresent(Variable.class))
                .toArray(Field[]::new);
            SmartDashboard.putString("Fields",Arrays.toString(fields));
        for (Field f : fields) {
            f.setAccessible(true);
            Variable annot = f.getAnnotation(Variable.class);
            map.put(annot.key(),
                    new Property(
                            new Property.Callable(f,this,true),
                            annot.mutable() && !Modifier.isFinal(f.getModifiers()) ? new Property.Callable(f,this,false) : null,
                            f.getType()
                    )
            );
        }
        //FINALLY build everything
        for (Map.Entry<String,Property> entry : map.entrySet()) {
            entry.getValue().addToBuilder(builder,entry.getKey());
        }
    }
}