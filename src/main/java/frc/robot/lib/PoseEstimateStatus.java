// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class PoseEstimateStatus {
    private TagFoundState m_state = TagFoundState.EMPTY;
    private double m_ambiguity = -1.0;
    private int m_tagID = -1;
    private Pose2d m_estimatedPose = new Pose2d();
    public PoseEstimateStatus(TagFoundState _state, double _ambiguity, int _tagID){
      //  m_estimatedPose = _pose;

        m_state = _state;
        m_ambiguity = _ambiguity;
        m_tagID = _tagID;
    }
    public TagFoundState getState(){
        return m_state;
    }
    public double getAmbiguity(){
        return m_ambiguity;
    }
    public int getTagID(){
        return m_tagID;
    }
    public Pose2d getEstimatedPose(){
        return m_estimatedPose;
    }
}
