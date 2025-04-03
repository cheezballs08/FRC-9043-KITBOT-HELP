package frc.robot.constants;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

// I used the units library heavily but you guys can use whatever you want, just refactor it however you want.
public class DrivetrainConstants {
  // FOR CUSTOM ONLY!
  
  // Only motors on one side!
  public static final DCMotor DRIVE_MOTORS = DCMotor.getCIM(2);

  // Gearing for that side. 
  public static final double GEARING = 1.0;
  
  public static final MomentOfInertia MOI = KilogramSquareMeters.of(0);

  public static final Mass MASS = Kilograms.of(0);

  public static final Distance WHEEL_DIAMETER = Centimeters.of(0);

  public static final Distance TRACK_WIDTH = Centimeters.of(66.04);

  public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);

  public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(2);

  // KITBOT, CHANGE THESE TO YOUR DESIRED VALUES!

  public static final KitbotMotor KITBOT_MOTORS = KitbotMotor.kDualCIMPerSide;

  public static final KitbotGearing KITBOT_GEARING = KitbotGearing.k8p45; 

  public static final KitbotWheelSize KITBOT_WHEEL_SIZE = KitbotWheelSize.kEightInch;

  public static final Pose2d BLUE_START = new Pose2d(
    Meters.of(7.30), 
    Meters.of(6.18), 
    Rotation2d.fromDegrees(180)
  );

  public static final Pose2d INITIAL_POSE = BLUE_START;

  /*public static RobotConfig CONFIG;
  
  static {
    try {
      CONFIG = RobotConfig.fromGUISettings();
    } catch (IOException | ParseException e) {
      e.printStackTrace();
    }
  }*/
}
