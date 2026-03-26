package frc.robot.utils;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

/**
 * Logs shooter test data to CSV file for analysis and lookup table creation.
 * Records one row per test sequence with aggregate statistics.
 */
public class ShooterTestLogger {
  private static final String CSV_FILE_PATH = "/home/lvuser/shooter_test_data.csv";
  private static final DateTimeFormatter TIMESTAMP_FORMAT = DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss");

  private final NetworkTable m_telemetryTable;
  private final IntegerPublisher m_sequenceNumberPub;
  private final IntegerPublisher m_ballsLoadedPub;
  private final IntegerPublisher m_ballsScoredPub;
  private final StringPublisher m_testStatusPub;

  private int m_sequenceNumber = 0;
  private boolean m_headerWritten = false;

  // Telemetry sampling buffers
  private List<Double> m_shooterRPMSamples = new ArrayList<>();
  private List<Double> m_indexerRPMSamples = new ArrayList<>();
  private List<Double> m_shooterCurrentSamples = new ArrayList<>();
  private List<Double> m_indexerCurrentSamples = new ArrayList<>();
  private List<Double> m_voltageSamples = new ArrayList<>();

  public ShooterTestLogger() {
    m_telemetryTable = NetworkTableInstance.getDefault().getTable("ShooterTest");
    m_sequenceNumberPub = m_telemetryTable.getIntegerTopic("Sequence Number").publish();
    m_ballsLoadedPub = m_telemetryTable.getIntegerTopic("Balls Loaded").publish();
    m_ballsScoredPub = m_telemetryTable.getIntegerTopic("Balls Scored").publish();
    m_testStatusPub = m_telemetryTable.getStringTopic("Test Status").publish();
  }

  /**
   * Start a new test sequence - clears sample buffers
   */
  public void startSequence() {
    m_shooterRPMSamples.clear();
    m_indexerRPMSamples.clear();
    m_shooterCurrentSamples.clear();
    m_indexerCurrentSamples.clear();
    m_voltageSamples.clear();
  }

  /**
   * Add telemetry samples during the shooting sequence
   */
  public void addSample(double shooterRPM, double indexerRPM,
                        double shooterCurrent, double indexerCurrent, double voltage) {
    m_shooterRPMSamples.add(shooterRPM);
    m_indexerRPMSamples.add(indexerRPM);
    m_shooterCurrentSamples.add(shooterCurrent);
    m_indexerCurrentSamples.add(indexerCurrent);
    m_voltageSamples.add(voltage);
  }

  /**
   * Log completed test sequence to CSV file
   */
  public void logSequence(TestSequenceData data) {
    try {
      BufferedWriter writer = new BufferedWriter(new FileWriter(CSV_FILE_PATH, true));

      // Write header if this is the first entry
      if (!m_headerWritten) {
        writer.write("Sequence_ID,Timestamp,Odometry_Distance_m,Measured_Distance_m,Balls_Loaded,Balls_Scored,Success_Rate,");
        writer.write("Target_RPM,Avg_RPM,Min_RPM,Max_RPM,SpinUp_Time_s,");
        writer.write("Indexer_Target_RPM,Indexer_Avg_RPM,Indexer_Min_RPM,Indexer_Max_RPM,");
        writer.write("Shooter_Avg_Current,Shooter_Peak_Current,Indexer_Avg_Current,");
        writer.write("Voltage_Start,Voltage_Min,Voltage_Min_Time_s,Voltage_Avg,");
        writer.write("Sequence_Duration_s,Fire_Rate_balls_per_sec,Notes\n");
        m_headerWritten = true;
      }

      // Calculate statistics
      double avgShooterRPM = calculateAverage(m_shooterRPMSamples);
      double minShooterRPM = calculateMin(m_shooterRPMSamples);
      double maxShooterRPM = calculateMax(m_shooterRPMSamples);

      double avgIndexerRPM = calculateAverage(m_indexerRPMSamples);
      double minIndexerRPM = calculateMin(m_indexerRPMSamples);
      double maxIndexerRPM = calculateMax(m_indexerRPMSamples);

      double avgShooterCurrent = calculateAverage(m_shooterCurrentSamples);
      double peakShooterCurrent = calculateMax(m_shooterCurrentSamples);
      double avgIndexerCurrent = calculateAverage(m_indexerCurrentSamples);

      double voltageStart = m_voltageSamples.isEmpty() ? 0.0 : m_voltageSamples.get(0);
      double voltageMin = calculateMin(m_voltageSamples);
      double voltageAvg = calculateAverage(m_voltageSamples);
      double voltageMinTime = findMinTime(m_voltageSamples, data.sequenceDuration);

      double successRate = data.ballsLoaded > 0 ?
          (double) data.ballsScored / data.ballsLoaded * 100.0 : 0.0;
      double fireRate = data.sequenceDuration > 0 ?
          data.ballsLoaded / data.sequenceDuration : 0.0;

      // Write data row
      writer.write(String.format("%d,%s,%.3f,%.3f,%d,%d,%.1f,",
          m_sequenceNumber,
          data.timestamp,
          data.odometryDistance,
          data.measuredDistance,
          data.ballsLoaded,
          data.ballsScored,
          successRate));

      writer.write(String.format("%.0f,%.0f,%.0f,%.0f,%.3f,",
          data.targetRPM,
          avgShooterRPM,
          minShooterRPM,
          maxShooterRPM,
          data.spinUpTime));

      writer.write(String.format("%.0f,%.0f,%.0f,%.0f,",
          data.indexerTargetRPM,
          avgIndexerRPM,
          minIndexerRPM,
          maxIndexerRPM));

      writer.write(String.format("%.1f,%.1f,%.1f,",
          avgShooterCurrent,
          peakShooterCurrent,
          avgIndexerCurrent));

      writer.write(String.format("%.2f,%.2f,%.3f,%.2f,",
          voltageStart,
          voltageMin,
          voltageMinTime,
          voltageAvg));

      writer.write(String.format("%.3f,%.2f,%s\n",
          data.sequenceDuration,
          fireRate,
          data.notes));

      writer.close();

      // Update NetworkTables
      m_sequenceNumberPub.set(m_sequenceNumber);
      m_ballsLoadedPub.set(data.ballsLoaded);
      m_ballsScoredPub.set(data.ballsScored);
      m_testStatusPub.set("Logged sequence " + m_sequenceNumber);

      m_sequenceNumber++;

    } catch (IOException e) {
      System.err.println("Failed to write shooter test data: " + e.getMessage());
    }
  }

  /**
   * Update test status in NetworkTables
   */
  public void setStatus(String status) {
    m_testStatusPub.set(status);
  }

  /**
   * Get current sequence number
   */
  public int getSequenceNumber() {
    return m_sequenceNumber;
  }

  // Helper methods for statistics
  private double calculateAverage(List<Double> samples) {
    if (samples.isEmpty()) return 0.0;
    double sum = 0.0;
    for (double sample : samples) {
      sum += sample;
    }
    return sum / samples.size();
  }

  private double calculateMin(List<Double> samples) {
    if (samples.isEmpty()) return 0.0;
    double min = samples.get(0);
    for (double sample : samples) {
      if (sample < min) min = sample;
    }
    return min;
  }

  private double calculateMax(List<Double> samples) {
    if (samples.isEmpty()) return 0.0;
    double max = samples.get(0);
    for (double sample : samples) {
      if (sample > max) max = sample;
    }
    return max;
  }

  private double findMinTime(List<Double> samples, double totalDuration) {
    if (samples.isEmpty()) return 0.0;
    double minValue = calculateMin(samples);
    int minIndex = samples.indexOf(minValue);
    return (minIndex / (double) samples.size()) * totalDuration;
  }

  /**
   * Data structure for a completed test sequence
   */
  public static class TestSequenceData {
    public String timestamp;
    public double odometryDistance;
    public double measuredDistance;
    public int ballsLoaded;
    public int ballsScored;
    public double targetRPM;
    public double indexerTargetRPM;
    public double spinUpTime;
    public double sequenceDuration;
    public String notes;

    public TestSequenceData() {
      this.timestamp = LocalDateTime.now().format(TIMESTAMP_FORMAT);
      this.notes = "";
    }
  }
}
