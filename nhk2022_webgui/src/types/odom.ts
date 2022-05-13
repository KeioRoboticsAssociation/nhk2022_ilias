import { TopicInfo } from "./msgTypes";

export type OdomType = {
  header: {
    seq: number;
    stamp: number;
    // eslint-disable-next-line @typescript-eslint/naming-convention
    frame_id: string;
  };
  // eslint-disable-next-line @typescript-eslint/naming-convention
  child_frame_id: string;
  pose: {
    pose: {
      position: {
        x: number;
        y: number;
        z: number;
      };
      orientation: {
        x: number;
        y: number;
        z: number;
        w: number;
      };
    };
    covariance: number[];
  };
  twist: {
    twist: {
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
  };
};

export const odomInfo: TopicInfo = {
  name: "/odom",
  messageType: "nav_msgs/Odometry",
};
