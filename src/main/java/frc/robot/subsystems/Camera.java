// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.VisionConstants;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.utils.AprilCam;

public class Camera extends SubsystemBase {
  /** Creates a new Camera. */
private static Camera instance;
private AprilCam cam;

  public Camera() {
    this.cam = new AprilCam(VisionConstants.APRIL_CAM_NAME);
    //cam.update();
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


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
