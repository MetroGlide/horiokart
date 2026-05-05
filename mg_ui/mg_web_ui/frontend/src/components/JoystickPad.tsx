import { useRef, useEffect, useCallback } from "react";
import { FoxgloveClientHandle } from "../hooks/useFoxgloveClient";
import { useTeleop } from "../contexts/TeleopContext";
import { TOPICS } from "../ros/interfaces";

interface JoystickPadProps {
  client: FoxgloveClientHandle;
  onVelChange?: (linear: number, angular: number) => void;
}

const RADIUS = 52;
const THUMB_R = 16;
const PUBLISH_INTERVAL_MS = 100;

export default function JoystickPad({ client, onVelChange }: JoystickPadProps) {
  const { maxLinear, maxAngular } = useTeleop();
  const svgRef = useRef<SVGSVGElement>(null);
  const thumbRef = useRef<SVGCircleElement>(null);
  const velRef = useRef({ linear: 0, angular: 0 });
  const dragging = useRef(false);
  const intervalRef = useRef<ReturnType<typeof setInterval> | null>(null);

  const publishVel = useCallback(
    (linear: number, angular: number) => {
      if (client.status !== "connected") return;
      client.publish(TOPICS.CMD_VEL, "geometry_msgs/msg/Twist", {
        linear: { x: linear, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: angular },
      });
      onVelChange?.(linear, angular);
    },
    [client, onVelChange],
  );

  const startPublish = useCallback(() => {
    if (intervalRef.current) return;
    intervalRef.current = setInterval(() => {
      publishVel(velRef.current.linear, velRef.current.angular);
    }, PUBLISH_INTERVAL_MS);
  }, [publishVel]);

  const stopPublish = useCallback(() => {
    if (intervalRef.current) {
      clearInterval(intervalRef.current);
      intervalRef.current = null;
    }
    velRef.current = { linear: 0, angular: 0 };
    publishVel(0, 0);
    onVelChange?.(0, 0);
    if (thumbRef.current) {
      thumbRef.current.setAttribute("cx", String(RADIUS + THUMB_R));
      thumbRef.current.setAttribute("cy", String(RADIUS + THUMB_R));
    }
  }, [publishVel, onVelChange]);

  useEffect(() => {
    const svg = svgRef.current;
    if (!svg) return;

    const getRelPos = (e: PointerEvent) => {
      const rect = svg.getBoundingClientRect();
      const cx = rect.left + rect.width / 2;
      const cy = rect.top + rect.height / 2;
      return { dx: e.clientX - cx, dy: e.clientY - cy };
    };

    const onPointerDown = (e: PointerEvent) => {
      dragging.current = true;
      svg.setPointerCapture(e.pointerId);
      startPublish();
    };

    const onPointerMove = (e: PointerEvent) => {
      if (!dragging.current) return;
      const { dx, dy } = getRelPos(e);
      const dist = Math.min(Math.sqrt(dx * dx + dy * dy), RADIUS);
      const angle = Math.atan2(dy, dx);
      const clampedX = Math.cos(angle) * dist;
      const clampedY = Math.sin(angle) * dist;

      velRef.current = {
        linear: (-clampedY / RADIUS) * maxLinear,
        angular: (-clampedX / RADIUS) * maxAngular,
      };

      if (thumbRef.current) {
        thumbRef.current.setAttribute(
          "cx",
          String(RADIUS + THUMB_R + clampedX),
        );
        thumbRef.current.setAttribute(
          "cy",
          String(RADIUS + THUMB_R + clampedY),
        );
      }
    };

    const onPointerUp = () => {
      if (!dragging.current) return;
      dragging.current = false;
      stopPublish();
    };

    svg.addEventListener("pointerdown", onPointerDown);
    svg.addEventListener("pointermove", onPointerMove);
    svg.addEventListener("pointerup", onPointerUp);
    svg.addEventListener("pointercancel", onPointerUp);

    return () => {
      svg.removeEventListener("pointerdown", onPointerDown);
      svg.removeEventListener("pointermove", onPointerMove);
      svg.removeEventListener("pointerup", onPointerUp);
      svg.removeEventListener("pointercancel", onPointerUp);
    };
  }, [maxLinear, maxAngular, startPublish, stopPublish]);

  useEffect(() => {
    return () => {
      if (intervalRef.current) clearInterval(intervalRef.current);
    };
  }, []);

  const size = (RADIUS + THUMB_R) * 2;

  return (
    <div className="flex flex-col items-center select-none">
      <svg
        ref={svgRef}
        width={size}
        height={size}
        className="cursor-grab active:cursor-grabbing touch-none"
        style={{ userSelect: "none" }}
      >
        <circle
          cx={RADIUS + THUMB_R}
          cy={RADIUS + THUMB_R}
          r={RADIUS}
          fill="rgba(30,30,40,0.85)"
          stroke="rgba(100,120,180,0.6)"
          strokeWidth={2}
        />
        <line
          x1={RADIUS + THUMB_R}
          y1={THUMB_R}
          x2={RADIUS + THUMB_R}
          y2={size - THUMB_R}
          stroke="rgba(255,255,255,0.1)"
          strokeWidth={1}
        />
        <line
          x1={THUMB_R}
          y1={RADIUS + THUMB_R}
          x2={size - THUMB_R}
          y2={RADIUS + THUMB_R}
          stroke="rgba(255,255,255,0.1)"
          strokeWidth={1}
        />
        <circle
          ref={thumbRef}
          cx={RADIUS + THUMB_R}
          cy={RADIUS + THUMB_R}
          r={THUMB_R}
          fill="rgba(80,120,220,0.9)"
          stroke="rgba(140,170,255,0.8)"
          strokeWidth={2}
        />
      </svg>
      <span className="text-xs text-gray-500 mt-1">drag to drive</span>
    </div>
  );
}
