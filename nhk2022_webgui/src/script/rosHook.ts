// https://stackoverflow.com/questions/69491226/after-using-vitevue3-to-build-the-project-and-importing-roslibjs-an-error-is-r
import ROSLIB from "roslib";
import "roslib/build/roslib";
import { onMounted, ref, onUnmounted, reactive, computed, toRefs } from "vue";
import { TopicInfo } from "../types/msgTypes";

const roslib = window.ROSLIB;

const ros = new roslib.Ros({
  // url: "ws://localhost:9090",
});

export function connectRos(url: string) {
  ros.connect(url);
}

type RosStatus = {
  isConnected: boolean;
  isError: boolean;
  errorEvent?: Event;
};

export function useRosStatus() {
  const status = reactive<RosStatus>({
    isConnected: ros.isConnected,
    isError: false,
  });
  const connectonListener = computed(() => () => {
    status.isConnected = true;
    console.log("ros connected");
  });
  const closeListener = computed(() => () => {
    status.isConnected = false;
  });
  const errorListener = computed(() => (event: Event) => {
    status.isError = true;
    status.errorEvent = event;
  });
  onMounted(() => {
    ros.on("connection", connectonListener.value);
    ros.on("close", closeListener.value);
    ros.on("error", errorListener.value);
  });
  onUnmounted(() => {
    ros.off("connected", connectonListener.value);
    ros.off("close", closeListener.value);
    ros.off("error", errorListener.value);
  });
  return toRefs(status);
}

export const createTopic = <T extends object>(topicInfo: TopicInfo) =>
  new roslib.Topic<T>({
    ros,
    name: topicInfo.name,
    messageType: topicInfo.messageType,
  });

export const useSubscriber = <T>(topic: ROSLIB.Topic<T>) => {
  const subscribeData = ref<T>();
  onMounted(() => {
    topic.subscribe((msg) => {
      subscribeData.value = msg;
    });
  });
  onUnmounted(() => {
    topic.unsubscribe();
  });
  return subscribeData;
};

export const publishTopic = <T>(topic: ROSLIB.Topic<T>, data: T) => {
  topic.publish(data);
};
