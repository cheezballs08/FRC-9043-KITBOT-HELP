package frc.robot.subsystems.drivetrain.implementations;

import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.motorsims.SimulatedBattery;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class SimpleSimMotorController implements MotorController {

  private double voltage;

  @Override
  public void set(double speed) {
    this.voltage = speed * SimulatedBattery.getBatteryVoltage().in(Volts);
  }

  @Override
  public double get() {
    return this.voltage / SimulatedBattery.getBatteryVoltage().in(Volts);
  }

  @Override
  public void setInverted(boolean isInverted) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setInverted'");
  }

  @Override
  public boolean getInverted() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getInverted'");
  }

  @Override
  public void disable() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'disable'");
  }

  @Override
  public void stopMotor() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'stopMotor'");
  }
  
}