package frc.robot.auto.general;

import static org.wpilib.units.Units.Seconds;

import coppercore.parameter_tools.json.annotations.AfterJsonLoad;
import coppercore.parameter_tools.json.annotations.JSONExclude;
import org.wpilib.units.measure.Time;
import org.wpilib.command2.Command;
import org.wpilib.command2.DeferredCommand;
import org.wpilib.command2.WaitCommand;
import frc.robot.auto.AutoAction;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

// Class written with the help of Github Copilot auto-completions

/**
 * A NetworkConfigurableWait is a Wait action that can be configured in real-time through the
 * network.
 */
public class NetworkConfigurableWait extends AutoAction {
  private String name;
  private Time defaultDelay;
  @JSONExclude private LoggedNetworkNumber waitTimeSeconds;

  /** No-arg constructor retained for JSON deserialization. */
  public NetworkConfigurableWait() {}

  /** Authoring constructor used by the Java auto generator. */
  public NetworkConfigurableWait(String name, Time defaultDelay) {
    this.name = name;
    this.defaultDelay = defaultDelay;
  }

  @JSONExclude
  private static final Map<String, LoggedNetworkNumber> nameToLoggedNetworkNumber = new HashMap<>();

  @AfterJsonLoad
  public void initializeNetworkNumber() {
    this.waitTimeSeconds =
        nameToLoggedNetworkNumber.computeIfAbsent(
            name,
            name ->
                new LoggedNetworkNumber(
                    "NetworkConfigurableWait/" + name,
                    defaultDelay != null ? defaultDelay.in(Seconds) : 0.0));
  }

  @Override
  public Command toCommand(AutoActionContext data) {
    return new DeferredCommand(() -> new WaitCommand(Seconds.of(waitTimeSeconds.get())), Set.of());
  }
}
