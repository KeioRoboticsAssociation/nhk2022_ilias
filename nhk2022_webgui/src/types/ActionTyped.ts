import "roslib/build/roslib";

const roslib = window.ROSLIB;

export type ActionStatus = {
  // eslint-disable-next-line @typescript-eslint/naming-convention
  goal_id: {
    stamp: {
      secs: number;
      nsecs: number;
    };
    id: string;
  };
  status: number;
  text: string;
};

export class GoalTyped<TGoal, TFeedback, TResult> extends roslib.Goal {
  // eslint-disable-next-line no-useless-constructor
  constructor(options: {
    actionClient: ROSLIB.ActionClient;
    goalMessage: TGoal;
  }) {
    super(options);
  }

  on(eventName: "feedback", callback: (event: TFeedback) => void): void;

  on(eventName: "result", callback: (event: TResult) => void): void;

  on(eventName: "status", callback: (event: ActionStatus) => void): void;

  on(
    eventName: "feedback" | "result" | "status",
    callback:
      | ((event: TFeedback) => void)
      | ((event: TResult) => void)
      | ((event: ActionStatus) => void)
  ) {
    super.on(eventName, callback);
  }
}
