import { TopicInfo } from "./msgTypes";

export type JoyType = {
  axes: number[];
  buttons: boolean[];
};

export const joyInfo: TopicInfo = {
  messageType: "sensor_msgs/Joy",
  name: "/joy",
};
