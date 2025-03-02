package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class TunablePIDController extends PIDController {
  LoggedNetworkNumber tunableP, tunableI, tunableD;

  public TunablePIDController(double kp, double ki, double kd, String path) {
    super(kp, ki, kd, 0.02);
    tunableP = new LoggedNetworkNumber(path + "P", kp);
    tunableI = new LoggedNetworkNumber(path + "I", ki);
    tunableD = new LoggedNetworkNumber(path + "D", kd);
  }

  public void update() {
    this.setP(tunableP.get());
    this.setI(tunableI.get());
    this.setD(tunableD.get());
  }
}
