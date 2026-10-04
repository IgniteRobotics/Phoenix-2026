package frc.robot.diagnostics;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.time.Duration;

/**
 * Logs the CTRE CAN inventory (device serial numbers) to the wpilog entry {@code
 * /Flashpoint/CANInventory}, so Flashpoint can track physical motors across swaps and robots.
 *
 * <p>Reads the Phoenix Diagnostics Server ({@code localhost:1250/?action=getdevices}) on a daemon
 * thread. Call {@link #start()} once after {@code DataLogManager.start()}, and {@link #recheck()}
 * from {@code disabledInit()}. It queries only while disabled (the query triggers a bus
 * enumeration), never blocks the robot loop, and never throws into robot code.
 */
public final class CanInventoryLogger {
  public static final String ENTRY_NAME = "/Flashpoint/CANInventory";

  private static final URI GET_DEVICES = URI.create("http://localhost:1250/?action=getdevices");
  private static final Duration REQUEST_TIMEOUT = Duration.ofSeconds(2);
  private static final long MAX_WAIT_MS = 60_000;
  private static final long INITIAL_BACKOFF_MS = 1_000;

  private static StringLogEntry entry;
  private static String lastLogged;
  private static boolean started;
  private static boolean running;

  private CanInventoryLogger() {}

  /** Starts the boot-time capture. Safe to call more than once. */
  public static synchronized void start() {
    if (started) {
      return;
    }
    started = true;
    entry = new StringLogEntry(DataLogManager.getLog(), ENTRY_NAME);
    launch();
  }

  /** Re-captures on entry to Disabled (catches pit swaps). Logs only if the inventory changed. */
  public static synchronized void recheck() {
    if (started) {
      launch();
    }
  }

  private static synchronized void launch() {
    if (running) {
      return;
    }
    running = true;
    Thread thread = new Thread(CanInventoryLogger::capture, "CanInventoryLogger");
    thread.setDaemon(true);
    thread.start();
  }

  private static void capture() {
    String payload;
    try {
      payload = poll();
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      payload = CanInventory.error("interrupted");
    } catch (RuntimeException e) {
      payload = CanInventory.error(e.getClass().getSimpleName() + ": " + e.getMessage());
    }
    record(payload);
  }

  private static String poll() throws InterruptedException {
    HttpClient client = HttpClient.newBuilder().connectTimeout(REQUEST_TIMEOUT).build();
    HttpRequest request = HttpRequest.newBuilder(GET_DEVICES).timeout(REQUEST_TIMEOUT).build();
    long deadline = System.currentTimeMillis() + MAX_WAIT_MS;
    long backoff = INITIAL_BACKOFF_MS;
    String lastError = "no response";
    while (System.currentTimeMillis() < deadline) {
      try {
        String body = client.send(request, HttpResponse.BodyHandlers.ofString()).body();
        if (CanInventory.hasDevices(body)) {
          return CanInventory.normalize(body);
        }
        lastError = "no devices enumerated yet";
      } catch (java.io.IOException e) {
        lastError = e.getClass().getSimpleName() + ": " + e.getMessage();
      }
      Thread.sleep(backoff);
      backoff = Math.min(backoff * 2, 8_000);
    }
    return CanInventory.error("timed out after " + MAX_WAIT_MS + " ms: " + lastError);
  }

  private static synchronized void record(String payload) {
    if (!payload.equals(lastLogged)) {
      entry.append(payload);
      lastLogged = payload;
    }
    running = false;
  }
}
