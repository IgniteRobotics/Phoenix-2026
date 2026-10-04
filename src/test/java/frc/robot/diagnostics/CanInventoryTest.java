package frc.robot.diagnostics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.junit.jupiter.api.Test;

class CanInventoryTest {
  private static final ObjectMapper MAPPER = new ObjectMapper();

  // Shape of a Phoenix Diagnostics Server getdevices response (field names from the
  // Phoenix 5-era example; verify against Phoenix 6 on a live robot).
  private static final String SAMPLE =
      """
      {"DeviceArray":[
        {"BootloaderRev":"0.2","CANbus":"rio","CurrentVers":"26.1.0.0","HardwareRev":"1.1",
         "ID":11,"ManDate":"Jan 9, 2025","Model":"Talon FX","Name":"FL Drive",
         "SerialNo":"6E9415C3394C4853","Vendor":"VEX Robotics"},
        {"CANbus":"6E9415C3394C485320202050101C18FF","ID":21,"Model":"Talon FX",
         "SerialNo":"ABCDEF0123456789"}
      ],"GeneralReturn":{"Error":0}}
      """;

  private static JsonNode parse(String json) throws Exception {
    return MAPPER.readTree(json);
  }

  @Test
  void normalizesDevicesToSchemaV1() throws Exception {
    JsonNode out = parse(CanInventory.normalize(SAMPLE));

    assertEquals(1, out.get("schema").asInt());
    assertEquals("phoenix-diag", out.get("source").asText());
    assertEquals(2, out.get("devices").size());

    JsonNode first = out.get("devices").get(0);
    assertEquals("Talon FX", first.get("model").asText());
    assertEquals("rio", first.get("bus").asText());
    assertEquals(11, first.get("id").asInt());
    assertEquals("6E9415C3394C4853", first.get("serial").asText());
    assertEquals("26.1.0.0", first.get("fw").asText());
    assertEquals("1.1", first.get("hw_rev").asText());
    assertEquals("0.2", first.get("boot_rev").asText());
    assertEquals("Jan 9, 2025", first.get("man_date").asText());
    assertEquals("FL Drive", first.get("name").asText());
  }

  @Test
  void missingOptionalFieldsBecomeNull() throws Exception {
    JsonNode second = parse(CanInventory.normalize(SAMPLE)).get("devices").get(1);

    assertEquals("ABCDEF0123456789", second.get("serial").asText());
    assertTrue(second.get("fw").isNull());
    assertTrue(second.get("man_date").isNull());
  }

  @Test
  void emptyDeviceArrayIsNotReady() {
    assertFalse(CanInventory.hasDevices("{\"DeviceArray\":[]}"));
    assertTrue(CanInventory.hasDevices(SAMPLE));
  }

  @Test
  void malformedResponseProducesErrorPayloadInsteadOfThrowing() throws Exception {
    JsonNode out = parse(CanInventory.normalize("<html>not json</html>"));

    assertEquals(1, out.get("schema").asInt());
    assertTrue(out.get("error").asText().startsWith("unparseable"));
    assertFalse(CanInventory.hasDevices("<html>not json</html>"));
  }

  @Test
  void errorPayloadEscapesMessage() throws Exception {
    JsonNode out = parse(CanInventory.error("connect \"refused\""));

    assertEquals("connect \"refused\"", out.get("error").asText());
  }
}
