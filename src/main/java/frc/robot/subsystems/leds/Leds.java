package frc.robot.subsystems.leds;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public class Leds extends SubsystemBase {
  private static Leds instance;

  // constants
  private static final int length = 300;

  private static Color shiftOn = Color.kGreen;
  private static Color shiftOff = Color.kBlue;
  private static Color disabled = Color.kRed;

  private static int limiterSpeed = 60;

  /* @AutoLogOutput */ private static String gameData = " ";

  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;

  private final ArrayList<BooleanSupplier> conds;
  private final ArrayList<Color> colors;

  private final SlewRateLimiter redLimiter = new SlewRateLimiter(limiterSpeed);
  private final SlewRateLimiter greenLimiter = new SlewRateLimiter(limiterSpeed);
  private final SlewRateLimiter blueLimiter = new SlewRateLimiter(limiterSpeed);

  private Leds() {
    leds = new AddressableLED(Constants.LED_PORT);
    leds.setLength(length);
    buffer = new AddressableLEDBuffer(length);
    leds.setData(buffer);
    leds.start();

    conds =
        new ArrayList<>(
            List.of(
                () -> DriverStation.isDisabled(),
                () -> isShiftOn(),
                () -> !isShiftOn(),
                () -> true));

    colors = new ArrayList<>(List.of(disabled, shiftOn, shiftOff, disabled));

    // Logger.recordOutput("gameData", gameData);
  }

  public static Leds getInstance() {
    if (instance == null) {
      instance = new Leds();
    }
    return instance;
  }

  @Override
  public void periodic() {
    if (gameData.charAt(0) != 'R' && gameData.charAt(0) != 'B') {
      String gameMessage = DriverStation.getGameSpecificMessage();
      if (gameMessage.length() > 0) {
        gameData = gameMessage.substring(0, 1);
        // Logger.recordOutput("gameData", gameData);
      }
    }

    // find first true boolean in conds
    int active = -1;
    int index = 0;
    while (active < 0) {
      if (conds.get(index).getAsBoolean()) {
        active = index;
      }
      index++;
    }

    for (int i = 0; i < length; i++) {
      buffer.setRGB(
          i,
          (int) (redLimiter.calculate(colors.get(active).red * 255)),
          (int) (greenLimiter.calculate(colors.get(active).green * 255)),
          (int) (blueLimiter.calculate(colors.get(active).blue * 255)));
    }

    leds.setData(buffer);
  }

  public static boolean isShiftOn() {
    if (DriverStation.isAutonomous()) {
      return true;
    }
    double timeSec = DriverStation.getMatchTime();
    // System.out.println("time: " + timeSec);
    if (timeSec > 130.0 || timeSec < 30.0) {
      return true;
    }
    // move time for easier arithmetic
    timeSec -= 30;
    timeSec /= 25;
    return (gameData.charAt(0) == getAllianceChar()
        && (timeSec == 0 || timeSec == 2 /* shifts 4 and 2, respectively */));
  }

  public static char getAllianceChar() {
    return DriverStation.getAlliance().get() == Alliance.Blue ? 'B' : 'R';
  }
}
