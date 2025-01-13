// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.VisionConstants;

/** Add your docs here. */
public class AprilCam {

    private PhotonCamera camera;
    private PhotonPipelineResult result;
    private static AprilCam instance;
    private PhotonTrackedTarget desiredTarget;
    
    

     public AprilCam(String name, Translation3d position, Rotation3d angle){
        this.camera = new PhotonCamera(name);
     }

     //simple constructor
     public AprilCam(String name) {
        this(name,new Translation3d(), new Rotation3d());
    }

     public static AprilCam getInstance() {
        if (instance == null) {
          instance = new AprilCam(VisionConstants.APRIL_CAM_NAME);
        }
        return instance;
      }

      public void update() {
        this.result = camera.getLatestResult();
    }
}
