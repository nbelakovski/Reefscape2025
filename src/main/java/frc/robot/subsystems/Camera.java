// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.VisionConstants;
import frc.robot.utils.AprilCam;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Camera extends SubsystemBase {
  /** Creates a new Camera. */
  private AprilCam cam;
  private static Camera instance;


  public Camera() {
    this.cam = new AprilCam(VisionConstants.APRIL_CAM_NAME);
    cam.update();
  }

  public static Camera getInstance(){
    if(instance == null){
      instance = new Camera();
    }
      return instance;
  }
  
  public AprilCam getCam() {
    return cam;
  }

  public PhotonTrackedTarget getDesiredTarget(int target) {
    return cam.getDesiredTarget(target);
  }

  public double getDesiredX(PhotonTrackedTarget target) {
    return cam.getDesiredX(target);
  }

  public double getX(){
    return cam.getX();
  }

  public double getDesiredY(PhotonTrackedTarget target) {
    return cam.getDesiredY(target);
  }

  public double getY(){
    return cam.getY();
  }

  public double getZ(){
    return cam.getZ();
  }
  public boolean hasTarget() {
    return cam.hasTarget();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
