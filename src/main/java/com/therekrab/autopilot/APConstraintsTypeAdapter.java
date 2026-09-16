package com.therekrab.autopilot;

import com.google.gson.JsonParseException;
import com.google.gson.TypeAdapter;
import com.google.gson.stream.JsonReader;
import com.google.gson.stream.JsonToken;
import com.google.gson.stream.JsonWriter;
import java.io.IOException;

/** Gson adapter that reconstructs {@link APConstraints} and its derived final fields. */
public final class APConstraintsTypeAdapter extends TypeAdapter<APConstraints> {
  @Override
  public void write(JsonWriter out, APConstraints constraints) throws IOException {
    if (constraints == null) {
      out.nullValue();
      return;
    }

    out.beginObject();
    out.name("velocity").value(constraints.velocity);
    out.name("acceleration").value(constraints.acceleration);
    out.name("jerk").value(constraints.jerk);
    out.endObject();
  }

  @Override
  public APConstraints read(JsonReader in) throws IOException {
    if (in.peek() == JsonToken.NULL) {
      in.nextNull();
      return null;
    }

    Double velocity = null;
    Double acceleration = null;
    Double jerk = null;

    in.beginObject();
    while (in.hasNext()) {
      switch (in.nextName()) {
        case "velocity" -> velocity = in.nextDouble();
        case "acceleration" -> acceleration = in.nextDouble();
        case "jerk" -> jerk = in.nextDouble();
        default -> in.skipValue();
      }
    }
    in.endObject();

    if (velocity == null || acceleration == null || jerk == null) {
      throw new JsonParseException(
          "APConstraints requires velocity, acceleration, and jerk fields");
    }

    return new APConstraints(velocity, acceleration, jerk);
  }
}
