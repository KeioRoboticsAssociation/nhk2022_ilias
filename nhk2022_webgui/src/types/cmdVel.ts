import { TopicInfo } from "./msgTypes";

export type CmdVelPacket = {
  linear: {
    x: number;
    y: number;
    z: number;
  };
  angular: {
    x: number;
    y: number;
    z: number;
  };
};

export const cmdVelInfo: TopicInfo = {
  name: "/cmd_vel",
  messageType: "geometry_msgs/Twist",
};
