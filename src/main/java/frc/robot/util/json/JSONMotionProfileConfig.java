package frc.robot.util.json;

import coppercore.parameter_tools.json.helpers.JSONObject;
import coppercore.wpilib_interface.subsystems.motors.profile.MotionProfileConfig;
import coppercore.wpilib_interface.subsystems.motors.profile.MutableMotionProfileConfig;
import org.wpilib.units.AngularAccelerationUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Per;
import org.wpilib.units.measure.Velocity;
import java.lang.reflect.Constructor;

public class JSONMotionProfileConfig extends JSONObject<MotionProfileConfig> {
  boolean isMutable;
  AngularVelocity maxVelocity;
  AngularAcceleration maxAcceleration;
  Velocity<AngularAccelerationUnit> maxJerk;
  Per<VoltageUnit, AngularVelocityUnit> expoKV;
  Per<VoltageUnit, AngularAccelerationUnit> expoKA;

  public JSONMotionProfileConfig(MotionProfileConfig motionProfileConfig) {
    super(motionProfileConfig);

    this.isMutable = motionProfileConfig instanceof MutableMotionProfileConfig;
    this.maxVelocity = motionProfileConfig.getMaxVelocity();
    this.maxAcceleration = motionProfileConfig.getMaxAcceleration();
    this.maxJerk = motionProfileConfig.getMaxJerk();
    this.expoKV = motionProfileConfig.getExpoKv();
    this.expoKA = motionProfileConfig.getExpoKa();
  }

  @Override
  public MotionProfileConfig toJava() {
    if (isMutable) {
      return MotionProfileConfig.mutable(maxVelocity, maxAcceleration, maxJerk, expoKV, expoKA);
    } else {
      return MotionProfileConfig.immutable(maxVelocity, maxAcceleration, maxJerk, expoKV, expoKA);
    }
  }

  public static Constructor<JSONMotionProfileConfig> getConstructor() throws NoSuchMethodException {
    return JSONMotionProfileConfig.class.getConstructor(MotionProfileConfig.class);
  }
}
