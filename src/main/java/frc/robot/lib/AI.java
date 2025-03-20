// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import java.util.HashMap;

/** Add your docs here. */
public class AI {
    static RobotAlignStates m_previousRobotAlignState = RobotAlignStates.BACK;
    static CoralLiftState m_previousCoralLiftState = CoralLiftState.START;
    static AprilTagAlignState m_previousAprilTagAlignState = AprilTagAlignState.LEFT;
    public static void initData(){
        AI.ReefModel.dataModel.clear();
        AI.ReefModel.dataModel.put("BACK", new ReefSideModel());
        AI.ReefModel.dataModel.put("BACK_RIGHT", new ReefSideModel());
        AI.ReefModel.dataModel.put("BACK_LEFT", new ReefSideModel());
        AI.ReefModel.dataModel.put("FRONT", new ReefSideModel());
        AI.ReefModel.dataModel.put("FRONT_RIGHT", new ReefSideModel());
        AI.ReefModel.dataModel.put("FRONT_LEFT", new ReefSideModel());
    }

    public static class StateInput {
        public static void setState(RobotAlignStates _state) {
             ReefModel.robotAlignState = _state;
             ReefModel.updateModel();
             StateOutput.setOutputData();
             m_previousRobotAlignState = ReefModel.robotAlignState;
             m_previousCoralLiftState = ReefModel.coralLiftState;
             m_previousAprilTagAlignState = ReefModel.aprilTagAlignState;
        }
    }

    public static class ReefModel {
        public static HashMap<String, ReefSideModel> dataModel = new HashMap<>();

        public static CoralLiftState coralLiftState = CoralLiftState.L3;
        public static AprilTagAlignState aprilTagAlignState = AprilTagAlignState.LEFT;
        public static RobotAlignStates robotAlignState = RobotAlignStates.BACK;

        public static void updateModel() {
            ReefSideModel rsm = dataModel.get(robotAlignState.toString());
            if (rsm != null) {
                if (!rsm.Level3R) {
                    coralLiftState = CoralLiftState.L3;
                    aprilTagAlignState = AprilTagAlignState.RIGHT;
                    rsm.Level3R = true;
                } else if (!rsm.Level3L) {
                    coralLiftState = CoralLiftState.L3;
                    aprilTagAlignState = AprilTagAlignState.LEFT;
                    rsm.Level3L = true;
                } else if (!rsm.Level2R) {
                    coralLiftState = CoralLiftState.L2;
                    aprilTagAlignState = AprilTagAlignState.RIGHT;
                    rsm.Level3L = true;
                } else if (!rsm.Level2L) {
                    coralLiftState = CoralLiftState.L2;
                    aprilTagAlignState = AprilTagAlignState.LEFT;
                    rsm.Level2L = true;
                }

            }
        }
        public static void resetReefState(AprilTagAlignState _aprilTagAlignState){
            ReefSideModel rsm = dataModel.get(m_previousRobotAlignState.toString());
            rsm.resetValue(m_previousCoralLiftState, _aprilTagAlignState);
        }
        public static void setReefState(RobotAlignStates _robotAlignState, CoralLiftState _coralLiftState, AprilTagAlignState _aprilTagAlignState ){
            ReefSideModel rsm = dataModel.get(_robotAlignState.toString());
            rsm.setValue(_coralLiftState, _aprilTagAlignState);
        }   
    }
    public static class StateOutput{
        public static void setOutputData(){
            g.ROBOT.drive.setTargetRobotAngle(AI.ReefModel.robotAlignState);
            g.VISION.aprilTagAlignState = AI.ReefModel.aprilTagAlignState;
            g.CORALLIFT.state = AI.ReefModel.coralLiftState;
            
        }
    }
    public static class ReefSideModel{
        public boolean Level2R = false;
        public boolean Level3R = false;
        public boolean Level2L = false;
        public boolean Level3L = false;

        public void resetValue(CoralLiftState _coralLiftState, AprilTagAlignState _aprilTagAlignState){
            
            switch (_coralLiftState) {
                case L2:
                    switch (_aprilTagAlignState) {
                        case LEFT:
                            Level2L = false;
                            break;
                        case RIGHT:
                            Level2R = false;
                            break;
                        default:
                            break;
                    }
                    break;
                case L3:
                    switch (_aprilTagAlignState) {
                        case LEFT:
                            Level3L = false;
                            break;
                        case RIGHT:
                            Level3R = false;
                            break;
                        default:
                            break;
                    }
                    break;
                case START:
                    break;
                default:
                    break;
    
            }
        }
        public void setValue(CoralLiftState _coralState, AprilTagAlignState _aprilState) {
            switch (_coralState) {
                case L2:
                    switch (_aprilState) {
                        case LEFT:
                            Level2L = true;
                            break;
                        case RIGHT:
                            Level2R = true;
                            break;
                        default:
                            break;
                    }
                    break;
                case L3:
                    switch (_aprilState) {
                        case LEFT:
                            Level3L = true;
                            break;
                        case RIGHT:
                            Level3R = true;
                            break;
                        default:
                            break;
                    }
                    break;
                case START:
                    break;
                default:
                    break;
    
            }
        }
    }
}
