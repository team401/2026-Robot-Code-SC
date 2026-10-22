package frc.robot.constants;

import coppercore.parameter_tools.json.annotations.JSONExclude;
import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.system.Filesystem;
import java.io.IOException;
import java.nio.file.Path;

/**
 * AprilTagConstants provides a constant for deciding between AndyMark and Welded fields, and to
 * easily get the AprilTagFieldLayout from that choice.
 */
public class AprilTagConstants {
  public enum FieldType {
    ANDYMARK("2026-rebuilt-andymark.json"),
    WELDED("2026-rebuilt-welded.json");

    private final String jsonFilename;

    FieldType(String jsonFilename) {
      this.jsonFilename = jsonFilename;
    }

    public String getJsonFilename() {
      return jsonFilename;
    }
  }

  public final FieldType fieldType = FieldType.WELDED;

  @JSONExclude private Field cachedLayout;

  public Field getTagLayout() {
    if (cachedLayout == null) {
      cachedLayout = Fields.FRC_2026_REBUILT_ANDY_MARK.loadField();
    }
    return cachedLayout;
  }
}
