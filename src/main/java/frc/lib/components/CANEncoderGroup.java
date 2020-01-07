package frc.lib.components;

import java.util.ArrayList;

import com.revrobotics.CANEncoder;

public class CANEncoderGroup {
  private ArrayList<CANEncoder> encoders;

  public CANEncoderGroup(CANEncoder... encoders) {
    for (var encoder : encoders) {
      this.encoders.add(encoder);
    }
  }

  public void addEncoder(CANEncoder encoder) {
    encoders.add(encoder);
  }

  public void setDistancePerPulse(double factor) {
    encoders.forEach(e -> e.setPositionConversionFactor(factor));
    encoders.forEach(e -> e.setVelocityConversionFactor(factor));
  }

  public double getPosition() {
    double sum = 0.0d;
    for (var encoder : encoders) {
      sum += encoder.getPosition();
    }
    return sum / encoders.size();
  }

  public double getVelocity() {
    double sum = 0.0d;
    for (var encoder : encoders) {
      sum += encoder.getVelocity();
    }
    return sum / encoders.size();
  }

  public void reset() {
    reset(0);
  }

  public void reset(double position) {
    encoders.forEach(e -> e.setPosition(position));
  }
}