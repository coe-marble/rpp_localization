"""Hold bag playback until the benchmark's ROS endpoints have been discovered."""

import time

import rclpy
from rclpy.qos import QoSCompatibility, QoSProfile, ReliabilityPolicy, qos_check_compatible
from rosbag2_interfaces.srv import IsPaused, Resume
from rosgraph_msgs.msg import Clock


def _missing_endpoints(node, filter_names, sensor_topics):
    missing = []
    for topic in ["/clock", *sensor_topics]:
        publishers = [endpoint for endpoint in node.get_publishers_info_by_topic(topic)
                      if endpoint.node_name == "rosbag2_player" and endpoint.node_namespace == "/"]
        if not publishers:
            missing.append(f"bag publisher on {topic}")
        subscribers = node.get_subscriptions_info_by_topic(topic)
        for name in filter_names:
            matches = [endpoint for endpoint in subscribers
                       if endpoint.node_name == "filter" and endpoint.node_namespace == f"/{name}"]
            if not matches:
                missing.append(f"/{name}/filter subscription to {topic}")
            elif publishers and not any(
                    qos_check_compatible(publisher.qos_profile, subscriber.qos_profile)[0]
                    != QoSCompatibility.ERROR
                    for publisher in publishers for subscriber in matches):
                raise RuntimeError(f"Incompatible bag/filter QoS on {topic} for /{name}/filter")

    for name in filter_names:
        topic = f"/{name}/odometry/filtered"
        if not any(endpoint.node_name == "filter" and endpoint.node_namespace == f"/{name}"
                   for endpoint in node.get_publishers_info_by_topic(topic)):
            missing.append(f"filter publisher on {topic}")
    return missing


def _call_service(node, client, request, check_running):
    future = client.call_async(request)
    deadline = time.monotonic() + 10.0
    while not future.done():
        check_running()
        if time.monotonic() >= deadline:
            client.remove_pending_request(future)
            raise RuntimeError(f"Timed out calling {client.srv_name}")
        rclpy.spin_once(node, timeout_sec=0.05)
    return future.result()


def resume_when_ready(node, filter_names, sensor_topics, start_time_ns, *,
                      check_running, before_resume):
    """Start at the first recorded timestamp, after clock and sensor discovery.

    The player's wall timer publishes /clock while paused. This lets filters
    finish their simulated-time initialization without losing sensor messages.
    """
    clock_ns = None

    def receive_clock(message):
        nonlocal clock_ns
        clock_ns = message.clock.sec * 1_000_000_000 + message.clock.nanosec

    clock_subscription = node.create_subscription(
        Clock, "/clock", receive_clock,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    paused_client = node.create_client(IsPaused, "/rosbag2_player/is_paused")
    resume_client = node.create_client(Resume, "/rosbag2_player/resume")
    try:
        deadline = time.monotonic() + 30.0
        ready_since = None
        next_endpoint_check = 0.0
        while time.monotonic() < deadline:
            check_running()
            rclpy.spin_once(node, timeout_sec=0.05)
            # /clock can arrive at 1 kHz; graph queries only need to run at 10 Hz.
            if time.monotonic() < next_endpoint_check:
                continue
            next_endpoint_check = time.monotonic() + 0.1
            missing = _missing_endpoints(node, filter_names, sensor_topics)
            if clock_ns != start_time_ns:
                missing.append(f"initial /clock = {start_time_ns} ns (received {clock_ns})")
            for client in (paused_client, resume_client):
                if not client.service_is_ready():
                    missing.append(client.srv_name)
            if missing:
                ready_since = None
                continue
            if ready_since is None:
                ready_since = time.monotonic()
            # Keep time frozen while DDS finishes matching the discovered endpoints.
            if time.monotonic() - ready_since >= 0.5:
                break
        else:
            waiting_for = "; ".join(missing) or "DDS discovery to settle"
            raise RuntimeError(f"Bag startup timed out waiting for: {waiting_for}")

        response = _call_service(node, paused_client, IsPaused.Request(), check_running)
        if not response.paused:
            raise RuntimeError("Bag playback advanced before the filters were ready")
        node.get_logger().info("Clock and filter subscriptions ready; resuming bag playback.")
        # CPU baselines exclude the startup barrier, and precede the first sensor message.
        before_resume()
        response = _call_service(node, resume_client, Resume.Request(), check_running)
        # Earlier rosbag2 versions return an empty Resume response.
        if getattr(response, "return_code", 0) != 0:
            raise RuntimeError(f"Bag resume failed: {response.error_string}")
    finally:
        node.destroy_subscription(clock_subscription)
        node.destroy_client(paused_client)
        node.destroy_client(resume_client)
