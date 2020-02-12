/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.subsystems;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Limelight implements Subsystem {
  private NetworkTable nt = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx = nt.getEntry("tx");
  private NetworkTableEntry ta = nt.getEntry("ta");
  private NetworkTableEntry tv = nt.getEntry("tv");
  private NetworkTableEntry tshort = nt.getEntry("tshort");
  private NetworkTableEntry ty = nt.getEntry("ty");
  private NetworkTableEntry ts = nt.getEntry("ts");
  private NetworkTableEntry pipeline = nt.getEntry("pipeline");

  private double distance;

  public enum Target {
    POWERCELL, HIGH, LOW, OLD;
  }

  private static Limelight _instance = new Limelight();

  public static Limelight getInstance() {
    return _instance;
  }

  private Limelight() {
  }

  public double getAngle1() {
    return tx.getDouble(0);
  }

  public double getTA() {
    return ta.getDouble(0);
  }

  public double getBoxShortLength() {
    return tshort.getDouble(0);
  }

  public double getSkew() {
    return ts.getDouble(0);
  }

  public double getYAng() {
    return ty.getDouble(0);
  }

  public double getDistanceToTarget(Target obj) {
    Target target = obj;
    switch (target) {
    default:
      distance = 0;
    case POWERCELL:
      double hypot = 63.971 * Math.pow(ta.getDouble(0), -0.461);
      distance = hypot > 8.5 ? Math.sqrt(hypot * hypot - 8.5 * 8.5) : 0;
      break;
    case HIGH:
      // distance = 214.81 * Math.pow(ta.getDouble(0), -0.418);
      // distance = Math.sqrt(Math.pow(9448.3 * Math.pow(tshort.getDouble(0), -0.904),
      // 2) - Math.pow(94, 2));
      distance = 86.25 / Math.tan(Math.toRadians(6.42 + ty.getDouble(0)));
      break;
    case LOW:
      distance = 0;
      break;
    }
    return distance;
  }

  public void setPipeline(double pipe) {
    pipeline.setDouble(pipe);
  }

  public boolean getHasTarget() {
    return tv.getDouble(0.0) != 0.0;
  }

  public void setZoom(boolean force){
    if (force){
      setPipeline(0);
    } else if (getHasTarget()){
      if (Math.abs(getAngle1()) <= 11.5 && getYAng() <= 11.7){
        setPipeline(1);
      } else{
        setPipeline(0);
      }
    } else{
      setPipeline(0);
    }
  }

  //This will always have you pointed at the vector currently to your target, getting the angle for a pinpoint target is much harder and not done here
  public RigidTransform2 getToTarget(Target target){
    return new RigidTransform2(Vector2.fromAngle(Rotation2.fromDegrees(getAngle1())).scale(getDistanceToTarget(target)), Rotation2.fromDegrees(getAngle1()));
  }
}
