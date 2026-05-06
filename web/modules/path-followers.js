function clamp(value, min, max) {
  return Math.min(Math.max(value, min), max);
}

function moveTowards(current, target, maxDelta) {
  if (Math.abs(target - current) <= maxDelta) {
    return target;
  }
  return current + Math.sign(target - current) * maxDelta;
}

function shortestAngleDelta(target, current) {
  return ((target - current + Math.PI) % (Math.PI * 2) + Math.PI * 2) % (Math.PI * 2) - Math.PI;
}

const go2PolylineFollower = {
  key: "go2Polyline",
  label: "Go2 Polyline Follower",
  begin({ plannerState, motionState, projectPointOntoPath }) {
    const projection = projectPointOntoPath(
      { x: motionState.x, z: motionState.z },
      plannerState.path,
      plannerState.polylineMetrics
    );
    const entryDistance = Math.hypot(projection.point.x - motionState.x, projection.point.z - motionState.z);
    return {
      pathProgress: projection.progress,
      autoEntryPoint: { x: projection.point.x, z: projection.point.z },
      autoEntryTargetIndex: clamp(projection.segmentIndex + 1, 1, plannerState.path.length - 1),
      autoSegmentIndex: clamp(projection.segmentIndex + 1, 1, plannerState.path.length - 1),
      autoSegmentPhase: entryDistance > 0.08 ? "startAlign" : "align",
    };
  },
  step(context) {
    const {
      plannerState,
      motionState,
      activeRobot,
      obstacleBounds,
      delta,
      turnSpeed,
      autoNavSpeed,
      sprintSpeed,
      halfWidth,
      halfHeight,
      resolveMotionWithSteps,
    } = context;

    const followerState = plannerState.followerState;
    const path = plannerState.path;
    const goalPoint = path[path.length - 1];
    const goalDistance = Math.hypot(goalPoint.x - motionState.x, goalPoint.z - motionState.z);

    if (followerState.autoSegmentIndex >= path.length && goalDistance < 0.12) {
      return { done: true, statusText: "Goal reached." };
    }

    const targetIndex = Math.min(followerState.autoSegmentIndex, path.length - 1);
    const isEntrySegment =
      followerState.autoEntryPoint && targetIndex === followerState.autoEntryTargetIndex;
    const previousPoint = isEntrySegment
      ? followerState.autoEntryPoint
      : path[Math.max(0, targetIndex - 1)];
    const targetPoint = path[targetIndex];
    const toTargetX = targetPoint.x - motionState.x;
    const toTargetZ = targetPoint.z - motionState.z;
    const waypointDistance = Math.hypot(toTargetX, toTargetZ);
    const isFinalWaypoint = targetIndex === path.length - 1;
    const segmentYaw = Math.atan2(
      -(targetPoint.z - previousPoint.z),
      targetPoint.x - previousPoint.x
    );
    const nextSegmentYaw = !isFinalWaypoint
      ? Math.atan2(
          -(path[targetIndex + 1].z - targetPoint.z),
          path[targetIndex + 1].x - targetPoint.x
        )
      : segmentYaw;
    const alignThreshold = 0.035;
    const turnThreshold = 0.04;
    const reachedWaypoint = waypointDistance < 0.04;

    const command = {
      directAutoMotion: true,
      directAutoSpeed: 0,
      forwardInput: 0,
      lateralInput: 0,
      turnInput: 0,
      poseForwardInput: 0,
      poseLateralInput: 0,
      poseTurnInput: 0,
      autoMoveDirection: null,
    };

    if (followerState.autoSegmentPhase === "startAlign") {
      const startPoint = followerState.autoEntryPoint ?? path[0];
      const startYaw = Math.atan2(-(startPoint.z - motionState.z), startPoint.x - motionState.x);
      const yawDelta = shortestAngleDelta(startYaw, motionState.yaw);
      const yawStep = clamp(yawDelta, -turnSpeed * 0.62 * delta, turnSpeed * 0.62 * delta);
      motionState.yaw += yawStep;
      command.turnInput = clamp(yawStep / Math.max(turnSpeed * delta, 1e-6), -0.62, 0.62);
      command.poseTurnInput = Math.sign(yawDelta || 1) * 0.82;
      command.directAutoSpeed = 0.42;

      if (Math.abs(yawDelta) < alignThreshold || Math.abs(yawStep - yawDelta) < 1e-4) {
        motionState.yaw = startYaw;
        followerState.autoSegmentPhase = "startMove";
      }
      return { command };
    }

    if (followerState.autoSegmentPhase === "startMove") {
      const startPoint = followerState.autoEntryPoint ?? path[0];
      const toStartX = startPoint.x - motionState.x;
      const toStartZ = startPoint.z - motionState.z;
      const startDistance = Math.hypot(toStartX, toStartZ);
      const moveSpeed = autoNavSpeed * clamp(startDistance / 0.45, 0.22, 0.72);
      const moveDistance = Math.min(moveSpeed * delta, startDistance);
      const dirX = toStartX / Math.max(startDistance, 1e-6);
      const dirZ = toStartZ / Math.max(startDistance, 1e-6);
      const targetX = clamp(motionState.x + dirX * moveDistance, -halfWidth, halfWidth);
      const targetZ = clamp(motionState.z + dirZ * moveDistance, -halfHeight, halfHeight);
      const resolved = resolveMotionWithSteps(
        targetX,
        targetZ,
        motionState.x,
        motionState.z,
        motionState.supportY,
        activeRobot.collisionRadius,
        activeRobot.stepHeight,
        obstacleBounds
      );
      motionState.x = resolved.x;
      motionState.z = resolved.z;
      if (motionState.isGrounded) {
        motionState.supportY = resolved.supportY;
      }
      command.forwardInput = 0.36;
      command.poseForwardInput = 0.78;
      command.directAutoSpeed = clamp(moveSpeed / sprintSpeed, 0.52, 0.72);

      if (startDistance < 0.04 || moveDistance >= startDistance - 1e-6) {
        motionState.x = startPoint.x;
        motionState.z = startPoint.z;
        if (typeof startPoint.y === "number" && motionState.isGrounded) {
          motionState.supportY = startPoint.y;
        }
        followerState.autoSegmentIndex = followerState.autoEntryTargetIndex;
        followerState.autoSegmentPhase = "align";
        command.forwardInput = 0;
      }
      return { command };
    }

    if (followerState.autoSegmentPhase === "align") {
      const yawDelta = shortestAngleDelta(segmentYaw, motionState.yaw);
      const yawStep = clamp(yawDelta, -turnSpeed * 0.62 * delta, turnSpeed * 0.62 * delta);
      motionState.yaw += yawStep;
      command.turnInput = clamp(yawStep / Math.max(turnSpeed * delta, 1e-6), -0.62, 0.62);
      command.poseTurnInput = Math.sign(yawDelta || 1) * 0.82;
      command.directAutoSpeed = 0.42;

      if (Math.abs(yawDelta) < alignThreshold || Math.abs(yawStep - yawDelta) < 1e-4) {
        motionState.yaw = segmentYaw;
        followerState.autoSegmentPhase = "move";
      }
      return { command };
    }

    if (followerState.autoSegmentPhase === "move") {
      const segmentDx = targetPoint.x - previousPoint.x;
      const segmentDz = targetPoint.z - previousPoint.z;
      const segmentLength = Math.hypot(segmentDx, segmentDz);
      command.autoMoveDirection = {
        x: segmentDx / Math.max(segmentLength, 1e-6),
        z: segmentDz / Math.max(segmentLength, 1e-6),
      };
      const moveSpeed = autoNavSpeed * clamp(waypointDistance / 0.45, 0.22, 0.72);
      const moveDistance = Math.min(moveSpeed * delta, waypointDistance);
      const targetX = clamp(motionState.x + command.autoMoveDirection.x * moveDistance, -halfWidth, halfWidth);
      const targetZ = clamp(motionState.z + command.autoMoveDirection.z * moveDistance, -halfHeight, halfHeight);
      const resolved = resolveMotionWithSteps(
        targetX,
        targetZ,
        motionState.x,
        motionState.z,
        motionState.supportY,
        activeRobot.collisionRadius,
        activeRobot.stepHeight,
        obstacleBounds
      );
      motionState.x = resolved.x;
      motionState.z = resolved.z;
      if (motionState.isGrounded) {
        motionState.supportY = resolved.supportY;
      }
      command.forwardInput = 0.36;
      command.poseForwardInput = 0.82;
      command.directAutoSpeed = clamp(moveSpeed / sprintSpeed, 0.56, 0.76);

      if (reachedWaypoint || moveDistance >= waypointDistance - 1e-6) {
        motionState.x = targetPoint.x;
        motionState.z = targetPoint.z;
        if (typeof targetPoint.y === "number" && motionState.isGrounded) {
          motionState.supportY = targetPoint.y;
        }
        if (isEntrySegment) {
          followerState.autoEntryPoint = null;
        }
        if (isFinalWaypoint) {
          followerState.autoSegmentIndex = path.length;
          command.forwardInput = 0;
          command.autoMoveDirection = null;
        } else {
          followerState.autoSegmentPhase = "turn";
          command.forwardInput = 0;
          command.autoMoveDirection = null;
        }
      }
      return { command };
    }

    if (followerState.autoSegmentPhase === "turn") {
      const yawDelta = shortestAngleDelta(nextSegmentYaw, motionState.yaw);
      const yawStep = clamp(yawDelta, -turnSpeed * 0.62 * delta, turnSpeed * 0.62 * delta);
      motionState.yaw += yawStep;
      command.turnInput = clamp(yawStep / Math.max(turnSpeed * delta, 1e-6), -0.62, 0.62);
      command.poseTurnInput = Math.sign(yawDelta || 1) * 0.88;
      command.directAutoSpeed = 0.48;

      if (Math.abs(yawDelta) < turnThreshold || Math.abs(yawStep - yawDelta) < 1e-4) {
        motionState.yaw = nextSegmentYaw;
        followerState.autoSegmentIndex += 1;
        followerState.autoSegmentPhase = "move";
      }
    }

    return { command };
  },
};

const turtlebotSmoothFollower = {
  key: "turtlebotSmooth",
  label: "TurtleBot Smooth Follower",
  begin({ plannerState, motionState, projectPointOntoPath }) {
    const projection = projectPointOntoPath(
      { x: motionState.x, z: motionState.z },
      plannerState.smoothPath,
      plannerState.smoothPathMetrics
    );
    return {
      pathProgress: projection.progress,
      autoSegmentIndex: 1,
      autoSegmentPhase: "align",
    };
  },
  step(context) {
    const {
      plannerState,
      motionState,
      delta,
      autoNavSpeed,
      projectPointOntoPath,
      samplePathAtProgress,
      autoCommandState,
    } = context;

    const currentPoint = { x: motionState.x, z: motionState.z };
    const projection = projectPointOntoPath(
      currentPoint,
      plannerState.smoothPath,
      plannerState.smoothPathMetrics
    );
    plannerState.followerState.pathProgress = Math.max(plannerState.followerState.pathProgress, projection.progress);

    const remaining = plannerState.smoothPathMetrics.totalLength - plannerState.followerState.pathProgress;
    const goalPoint = plannerState.smoothPath[plannerState.smoothPath.length - 1];
    const goalDistance = Math.hypot(goalPoint.x - motionState.x, goalPoint.z - motionState.z);

    if (remaining < 0.08 && goalDistance < 0.12) {
      return { done: true, statusText: "Goal reached." };
    }

    const lookAhead = clamp(0.34 + projection.distance * 1.15, 0.28, 0.7);
    const targetProgress = clamp(
      plannerState.followerState.pathProgress + lookAhead,
      0,
      plannerState.smoothPathMetrics.totalLength
    );
    const targetPoint = samplePathAtProgress(
      plannerState.smoothPath,
      plannerState.smoothPathMetrics,
      targetProgress
    );
    const toTargetX = targetPoint.x - motionState.x;
    const toTargetZ = targetPoint.z - motionState.z;
    const targetYaw = Math.atan2(-toTargetZ, toTargetX);
    const yawDelta = shortestAngleDelta(targetYaw, motionState.yaw);

    const desiredTurn = clamp(yawDelta * 1.5, -0.62, 0.62);
    const headingAbs = Math.abs(yawDelta);
    let desiredForward = 0.44;
    if (headingAbs > 0.75) {
      desiredForward = 0;
    } else if (headingAbs > 0.4) {
      desiredForward = 0.12;
    }

    const speedScale =
      goalDistance < 0.45
        ? clamp(goalDistance / 0.45, 0.16, 1)
        : clamp(1 - projection.distance * 0.95, 0.35, 1);
    const desiredSpeed = autoNavSpeed * speedScale;

    const maxForwardDelta = 0.8 * delta;
    const maxTurnDelta = 1.15 * delta;
    const maxSpeedDelta = 0.52 * delta;

    autoCommandState.forward = moveTowards(autoCommandState.forward, desiredForward, maxForwardDelta);
    autoCommandState.lateral = moveTowards(autoCommandState.lateral, 0, maxForwardDelta);
    autoCommandState.turn = moveTowards(autoCommandState.turn, desiredTurn, maxTurnDelta);
    autoCommandState.speed = moveTowards(autoCommandState.speed, desiredSpeed, maxSpeedDelta);

    return {
      command: {
        directAutoMotion: false,
        directAutoSpeed: 0,
        forwardInput: autoCommandState.forward,
        lateralInput: 0,
        turnInput: autoCommandState.turn,
        poseForwardInput: autoCommandState.forward,
        poseLateralInput: 0,
        poseTurnInput: autoCommandState.turn,
        autoMoveDirection: null,
        currentMoveSpeed: autoCommandState.speed,
      },
    };
  },
};

export const followerModules = {
  go2Polyline: go2PolylineFollower,
  turtlebotSmooth: turtlebotSmoothFollower,
};
