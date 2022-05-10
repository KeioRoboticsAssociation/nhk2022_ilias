/* eslint-disable @typescript-eslint/naming-convention */
import { ActionInfo } from "./msgTypes";

export const pursuitPathActionInfo: ActionInfo = {
  serverName: "/bezier_path_planning_pursuit",
  actionName: "bezier_path_planning_pursuit/PursuitPathAction",
};

export type PursuitPathResult = {
  result: boolean;
  position_x: number;
  position_y: number;
  position_theta: number;
};

export type PursuitPathFeedback = {
  reference_point: number;
  progress: number;
  vx: number;
  vy: number;
  omega: number;
  goal_x: number;
  goal_y: number;
};

export type PursuitPathGoal = {
  pathmode: number;
  direction: number;
};
