package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.motorsims.SimulatedBattery;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.util.DriveFeedforwards;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DrivetrainConstants;

public class SimDrivetrain extends SubsystemBase {

  DifferentialDrivetrainSim simulation;

  DifferentialDrive drive;

  SimpleSimMotorController leftMotor, rightMotor;

  /*PathFollowingController controller;*/

  public SimDrivetrain() {

    this.simulation = DifferentialDrivetrainSim.createKitbotSim(
      DrivetrainConstants.KITBOT_MOTORS, 
      DrivetrainConstants.KITBOT_GEARING, 
      DrivetrainConstants.KITBOT_WHEEL_SIZE, 
      null
    );

    this.simulation.setPose(DrivetrainConstants.INITIAL_POSE);

    this.leftMotor = new SimpleSimMotorController();
    this.rightMotor = new SimpleSimMotorController();

    this.drive = new DifferentialDrive(
      this.leftMotor::set, 
      this.rightMotor::set
    );

    // No PID since you guys have no sensors
    // Delete while removing pathplanner
    /*this.controller = new PPHolonomicDriveController(new PIDConstants(0), new PIDConstants(0));*/

    SimulatedBattery.addElectricalAppliances(() -> Amps.of(simulation.getCurrentDrawAmps()));

    // Delete this if you want to remove pathplanner
    /*AutoBuilder.configure(
      () -> new Pose2d(), 
      this::setSimPose, 
      () -> new ChassisSpeeds(), 
      this::drive, 
      this.controller, 
      // Delete this constant too!
      DrivetrainConstants.CONFIG, 
      () -> DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red),  
      this
    );*/
  }

  @Override
  public void periodic() {
    simulation.setInputs(
      leftMotor.get() * SimulatedBattery.getBatteryVoltage().in(Volts), 
      rightMotor.get() * SimulatedBattery.getBatteryVoltage().in(Volts)
    );

    simulation.update(0.02);

    DogLog.log("Drivetrain/SimPose", simulation.getPose());
  }

  public void drive(double speed, double turn) {
    this.drive.curvatureDrive(speed, turn, true);
  }

  public void drive(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    var wheelSpeeds = DrivetrainConstants.KINEMATICS.toWheelSpeeds(speeds);
    
    this.drive.tankDrive(
      wheelSpeeds.leftMetersPerSecond / 3.0, 
      wheelSpeeds.rightMetersPerSecond / 3.0, 
      false
    );
  }

  public void setSimPose(Pose2d pose) {
    this.simulation.setPose(pose);
  }
}