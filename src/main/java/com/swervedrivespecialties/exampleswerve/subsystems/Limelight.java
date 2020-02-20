/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.subsystems;

import com.swervedrivespecialties.exampleswerve.util.LogDataBE;
import com.swervedrivespecialties.exampleswerve.util.util;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Limelight implements Subsystem {
  private NetworkTable nt = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx = nt.getEntry("tx");
  private NetworkTableEntry ta = nt.getEntry("ta");
  private NetworkTableEntry tv = nt.getEntry("tv");
  private NetworkTableEntry tshort = nt.getEntry("tshort");
  private NetworkTableEntry tlong = nt.getEntry("tlong");
  private NetworkTableEntry thor = nt.getEntry("thor");
  private NetworkTableEntry tvert = nt.getEntry("tvert");
  private NetworkTableEntry ty = nt.getEntry("ty");
  private NetworkTableEntry ts = nt.getEntry("ts");
  private NetworkTableEntry ledMode = nt.getEntry("ledMode");
  private NetworkTableEntry pipeline = nt.getEntry("pipeline");

  private CAMERA_SLOT CUR_CAM_SLOT;
  private double distance;

  public enum Target {
    POWERCELL, HIGH, LOW, OLD;
  }

  private static Limelight _instance = new Limelight();

  public static Limelight getInstance() {
    return _instance;
  }

  private Limelight() {
    CUR_CAM_SLOT = CAMERA_SLOT.LIMELIGHT;
  }
  public double getAngle1() {
    return tx.getDouble(0);
  }
  public double getLedMode(){
    return ledMode.getDouble(0);
  }
  public void setLedMode(double mode){
    ledMode.forceSetDouble(mode);
  }
  public double getTA() {
    return ta.getDouble(0);
  }
  public double getBoxShortLength() {
    return tshort.getDouble(0);
  }
  public double getBoxLongLength(){
    return tlong.getDouble(0);
  }
  public double getHorBoxLength(){
    return thor.getDouble(0);
  }
  public double getVertBoxLength(){
    return tvert.getDouble(0);
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
      break;
    case POWERCELL:
      double hypot = 63.971 * Math.pow(ta.getDouble(0), -0.461);
      distance = hypot > 8.5 ? Math.sqrt(hypot * hypot - 8.5 * 8.5) : 0;
      break;
    case HIGH:
      distance = getDist(ty.getDouble(0));
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
    setPipeline(!force && getHasTarget() && Math.abs(getAngle1()) <= 11.5 && getYAng() <= 11.7 ? 1:0); 
  }
  public void toggleLEDS(){
    setPipeline(pipeline.getDouble(0) == 0 || pipeline.getDouble(0) == 1 ? 2: 0);
  }

  //be sure to change this method and the next one as distance changes
  private static double getDist(double tyVal){
    return 72.75 / Math.tan(Math.toRadians(21 + tyVal));
  }

  private static double inverseDist(double dist){
    return Math.toDegrees(Math.atan(72.75 / dist)) - 21;
  }

  //This will always have you pointed at the vector currently to your target, getting the angle for a pinpoint target is much harder and not done here
  public RigidTransform2 getToTarget(Target target){
    return new RigidTransform2(Vector2.fromAngle(Rotation2.fromDegrees(getAngle1())).scale(getDistanceToTarget(target)), Rotation2.fromDegrees(getAngle1()));
  }

  public void updateLogData(LogDataBE logData){
    if (getHasTarget()){
      logData.AddData("Distance", Double.toString(getDistanceToTarget(Target.HIGH)));
      logData.AddData("Y Angle", Double.toString(getYAng()));
      logData.AddData("Skew", Double.toString(getSkew()));
      logData.AddData("Angle 1", Double.toString(getAngle1()));
      logData.AddData("Box Short Length", Double.toString(getBoxShortLength()));
      logData.AddData("Box Long Length", Double.toString(getBoxLongLength()));
      logData.AddData("Horizontal Box Length", Double.toString(getHorBoxLength()));
      logData.AddData("Vertical Box Length", Double.toString(getVertBoxLength()));
      logData.AddData("Area", Double.toString(getTA()));
    }
  }

  private static class ConfidenceData{
    boolean hasTarget;
    double vert_ang;
    double horiz_ang;
    double box_ratio;
    double skew;

    public ConfidenceData(boolean ht, double va, double ha, double br, double sk){
      hasTarget = ht;
      vert_ang = va;
      horiz_ang = ha;
      box_ratio = br;
      skew = sk;
    }
  }

    ///////////////////////////////////////////////////////////////////////////////////////
   //////////////////////         CONFIDENCE CODE         ////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////

  private static final double kRangeLowDistance = 8.5 * 12; 
  private static final double kRangeHighDistance = 41 * 12;
  private static final double kRangeMinExcludedSkew = -65;
  private static final double kRangeMaxExcludedSkew = -25;

  private static final double kRangeMinVerticalAngle = inverseDist(kRangeHighDistance);
  private static final double kRangeMaxVerticalAngle = inverseDist(kRangeLowDistance);
  
  private static final double kOdometryLowDistance = 12 * 12; 
  private static final double kOdometryHighDistance = 32.5 * 12; 
  private static final double kOdometryMinExcludedSkew = -80;
  private static final double kOdometryMaxExcludedSkew = -20;
  private static final double kOdometryMinBoxRatio = 2.1;
  private static final double kOdometryMaxBoxRatio = 2.4;
  private static final double kOdometryMinHorizontalAngle = -18;
  private static final double kOdometryMaxHorizontalAngle = 18;

  private static final double kOdometryMinVerticalAngle = inverseDist(kOdometryHighDistance);
  private static final double kOdometryMaxVerticalAngle = inverseDist(kOdometryLowDistance);

  private boolean getHasRange(ConfidenceData cd){
    return cd.hasTarget && util.isInRange(cd.vert_ang, kRangeMinVerticalAngle, kRangeMaxVerticalAngle) && ! util.isInRange(cd.skew, kRangeMinExcludedSkew, kRangeMaxExcludedSkew);
  }

  private boolean getHasOdom(ConfidenceData cd){
    return cd.hasTarget && util.isInRange(cd.vert_ang, kOdometryMinVerticalAngle, kOdometryMaxVerticalAngle) && util.isInRange(cd.horiz_ang, kOdometryMinHorizontalAngle, kOdometryMaxHorizontalAngle)
    && util.isInRange(cd.box_ratio, kOdometryMinBoxRatio, kOdometryMaxBoxRatio) && !util.isInRange(cd.skew, kOdometryMinExcludedSkew, kOdometryMaxExcludedSkew);
  }

  private ConfidenceData getConfidenceData(){
    return new ConfidenceData(getHasTarget(), getYAng(), getAngle1(), getBoxShortLength() != 0 ? getBoxLongLength()/getBoxShortLength() : 0, getSkew());
  }

  public boolean hasRange(){
    return getHasRange(getConfidenceData());
  }

  public boolean hasOdom(){
    return getHasOdom(getConfidenceData());
  }

  private static final Translation2d BOT_TO_LL = new Translation2d(0, 0);

  public Translation2d getTargetToLL(){
    return util.transFromAngle(-getAngle1()).rotateBy(Rotation2d.fromDegrees(270)).rotateBy(Rotation2d.fromDegrees(-DrivetrainSubsystem.getInstance().getGyroAngle().toDegrees())).times(getDistanceToTarget(Target.HIGH));
  }

  private Translation2d getLLToBot(){
    return BOT_TO_LL.rotateBy(Rotation2d.fromDegrees(180).rotateBy(DrivetrainSubsystem.getInstance().getGyroRotation()));
  }

  public Translation2d getTargetToBot(){
    Vector2 vec = Vector2.fromAngle(DrivetrainSubsystem.getInstance().getGyroAngle().rotateBy(Rotation2.fromDegrees(-90)).rotateBy(Rotation2.fromDegrees(-getAngle1()))).scale(getDistanceToTarget(Target.HIGH)).rotateBy(Rotation2.fromDegrees(180));
    return new Translation2d(vec.x, vec.y);
  }

  public Translation2d getBotAngleToTarget(){
    return getTargetToBot().minus(new Translation2d( 0, 31.25)).rotateBy(Rotation2d.fromDegrees(180));
  }

  public Rotation2 locateFlavortownUSA(){
    return Vector2.fromAngle(DrivetrainSubsystem.getInstance().getGyroAngle().rotateBy(Rotation2.fromDegrees(-90)).rotateBy(Rotation2.fromDegrees(-getAngle1()))).scale(getDistanceToTarget(Target.HIGH)).add(-28.5, 0).getAngle().rotateBy(Rotation2.fromDegrees(90));
  }

  public enum CAMERA_SLOT{
    LIMELIGHT,
    INFEED,
    EXTRA,
    NONE;
  }

  private CAMERA_SLOT getNextCamera(CAMERA_SLOT slot){
    if (slot == CAMERA_SLOT.LIMELIGHT) {
      return CAMERA_SLOT.INFEED;
    } else if (slot == CAMERA_SLOT.INFEED) {
      return CAMERA_SLOT.EXTRA;
    } else if (slot == CAMERA_SLOT.EXTRA){
      return CAMERA_SLOT.LIMELIGHT;
    } else {
      return CAMERA_SLOT.NONE;
    }
  }

  private String getName(CAMERA_SLOT slot){
    if (slot == CAMERA_SLOT.LIMELIGHT) {
      return "LIMELIGHT";
    } else if (slot == CAMERA_SLOT.INFEED) {
      return "INFEED";
    } else if (slot == CAMERA_SLOT.EXTRA){
      return "EXTRA";
    } else {
      return "NONE";
    }
  }

  public void toggleCameraSlot(){
    CUR_CAM_SLOT = getNextCamera(CUR_CAM_SLOT);
  }

  public String getCam(){
    return getName(CUR_CAM_SLOT);
  }

  public void OutputToSDB(){
    SmartDashboard.putString("Camera Case", getCam());
  }
}