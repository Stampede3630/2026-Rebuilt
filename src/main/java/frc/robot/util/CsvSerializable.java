package frc.robot.util;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public interface CsvSerializable {
  /**
   * Get this object formatted as a CSV. Should be in the same order as headers specified by {@link
   * #getHeaders()}
   *
   * @return The object formatted as a line in a CSV file without a \n.
   */
  String toCsv();

  /**
   * Get the headers for this object as a CSV. Provides the column labels for data returned by
   * {@link #toCsv()}.
   *
   * @return A comma separated list of column names without a trailing \n.
   */
  String getHeaders();

  default void write(String path) throws IOException {
    File file = new File(path);

    try (FileWriter writer = new FileWriter(path)) {
      if (!file.exists()) {
        file.createNewFile();
        System.out.println("Creating new file at " + path);
        String headers = getHeaders() + "\n";
        writer.append(headers);
      }
      writer.append(toCsv() + "\n");
    } catch (IOException e) {
      throw new IOException(e);
    }
  }

  static void writeMany(String path, CsvSerializable... items) throws IOException {
    if (items.length == 0) {
      throw new IllegalArgumentException("Must provide at least 1 item.");
    }
    File file = new File(path);

    try (FileWriter writer = new FileWriter(path)) {
      if (!file.exists()) {
        file.createNewFile();
        System.out.println("Creating new file at " + path);
        String headers = items[0].getHeaders() + "\n";
        writer.append(headers);
      }
      for (CsvSerializable item : items) {
        writer.append(item.toCsv() + "\n");
      }
    } catch (IOException e) {
      throw new IOException(e);
    }
  }
}
