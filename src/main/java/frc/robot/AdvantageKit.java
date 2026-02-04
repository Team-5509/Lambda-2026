package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import org.littletonrobotics.junction.Logger;

public class AdvantageKit {
  private final XboxController controller;

  public AdvantageKit(int port) {
    controller = new XboxController(port);
  }

  public double getLeftY() {
    double value = controller.getLeftY();
    Logger.recordOutput("Controller/LeftY", value);
    return value;
  }

  public double getRightX() {
    double value = controller.getRightX();
    Logger.recordOutput("Controller/RightX", value);
    return value;
  }

  public boolean getAButton() {
    boolean value = controller.getAButton();
    Logger.recordOutput("Controller/AButton", value);
    return value;
  }
}
