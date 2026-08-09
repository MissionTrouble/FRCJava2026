package frc.robot.input;

import java.io.File;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import edu.wpi.first.wpilibj.Filesystem;

/** Saves/loads a recorded driver input session as a CSV file in the deploy directory. */
public final class InputRecording {
    private InputRecording() {}

    public static void save(List<DriverInputSample> samples, String fileName) {
        StringBuilder csv = new StringBuilder();
        for (DriverInputSample sample : samples) {
            csv.append(String.format(Locale.ROOT, "%.4f,%.4f,%.4f,%.4f,%.4f,%b,%b,%b,%b,%b,%b,%b,%b%n",
                sample.timestampSeconds(), sample.leftY(), sample.leftX(), sample.rightX(), sample.rightTrigger(),
                sample.xButton(), sample.yButton(), sample.startButton(), sample.backButton(),
                sample.leftBumper(), sample.rightBumper(), sample.povUp(), sample.povDown()));
        }

        File file = new File(Filesystem.getDeployDirectory(), fileName);
        try {
            Files.writeString(file.toPath(), csv.toString());
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    public static List<DriverInputSample> load(String fileName) {
        File file = new File(Filesystem.getDeployDirectory(), fileName);
        List<DriverInputSample> samples = new ArrayList<>();
        if (!file.exists()) {
            return samples;
        }

        try {
            for (String line : Files.readAllLines(file.toPath())) {
                if (line.isBlank()) {
                    continue;
                }
                String[] parts = line.split(",");
                samples.add(new DriverInputSample(
                    Double.parseDouble(parts[0]),
                    Double.parseDouble(parts[1]),
                    Double.parseDouble(parts[2]),
                    Double.parseDouble(parts[3]),
                    Double.parseDouble(parts[4]),
                    Boolean.parseBoolean(parts[5]),
                    Boolean.parseBoolean(parts[6]),
                    Boolean.parseBoolean(parts[7]),
                    Boolean.parseBoolean(parts[8]),
                    Boolean.parseBoolean(parts[9]),
                    Boolean.parseBoolean(parts[10]),
                    Boolean.parseBoolean(parts[11]),
                    Boolean.parseBoolean(parts[12])));
            }
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
        return samples;
    }
}
