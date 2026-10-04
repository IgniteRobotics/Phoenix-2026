package frc.robot.diagnostics;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

/**
 * Converts a Phoenix Diagnostics Server {@code getdevices} response into the Flashpoint CAN
 * inventory schema v1. Pure functions with no I/O, so they can be unit tested.
 *
 * <p>Schema v1: {@code {"schema":1,"source":"phoenix-diag","devices":[{"model","bus","id",
 * "serial","fw","hw_rev","boot_rev","man_date","name"}]}}. On failure: {@code
 * {"schema":1,"error":"..."}}. See flashpoint docs/rewrite/05-target-architecture.md section 4a.
 */
public final class CanInventory {
  public static final int SCHEMA_VERSION = 1;
  public static final String SOURCE = "phoenix-diag";

  private static final ObjectMapper MAPPER = new ObjectMapper();

  private CanInventory() {}

  /** True when the response parses and lists at least one device. */
  public static boolean hasDevices(String rawJson) {
    JsonNode devices = parseDeviceArray(rawJson);
    return devices != null && devices.size() > 0;
  }

  /** Normalizes a getdevices response. Never throws; returns an error payload instead. */
  public static String normalize(String rawJson) {
    JsonNode devices = parseDeviceArray(rawJson);
    if (devices == null) {
      return error("unparseable getdevices response");
    }
    ObjectNode out = MAPPER.createObjectNode();
    out.put("schema", SCHEMA_VERSION);
    out.put("source", SOURCE);
    ArrayNode list = out.putArray("devices");
    for (JsonNode device : devices) {
      ObjectNode d = list.addObject();
      copyText(device, "Model", d, "model");
      copyText(device, "CANbus", d, "bus");
      if (device.hasNonNull("ID")) {
        d.put("id", device.get("ID").asInt());
      } else {
        d.putNull("id");
      }
      copyText(device, "SerialNo", d, "serial");
      copyText(device, "CurrentVers", d, "fw");
      copyText(device, "HardwareRev", d, "hw_rev");
      copyText(device, "BootloaderRev", d, "boot_rev");
      copyText(device, "ManDate", d, "man_date");
      copyText(device, "Name", d, "name");
    }
    return out.toString();
  }

  /** Builds an error payload with a JSON-escaped message. */
  public static String error(String message) {
    ObjectNode out = MAPPER.createObjectNode();
    out.put("schema", SCHEMA_VERSION);
    out.put("error", message);
    return out.toString();
  }

  private static JsonNode parseDeviceArray(String rawJson) {
    try {
      JsonNode root = MAPPER.readTree(rawJson);
      JsonNode devices = root == null ? null : root.get("DeviceArray");
      return devices != null && devices.isArray() ? devices : null;
    } catch (Exception e) {
      return null;
    }
  }

  private static void copyText(JsonNode from, String fromKey, ObjectNode to, String toKey) {
    if (from.hasNonNull(fromKey)) {
      to.put(toKey, from.get(fromKey).asText());
    } else {
      to.putNull(toKey);
    }
  }
}
