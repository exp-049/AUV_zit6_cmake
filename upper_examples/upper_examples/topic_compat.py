"""Small ROS 2 helpers for the old ZIT6 and canonical AUV topic contracts."""

from __future__ import annotations

import threading
import time


class DualTopicPublisher:
    """Publish one message on both canonical and legacy topic names."""

    def __init__(self, node, message_type, topics, qos=10):
        self.topics = tuple(topics)
        self.publishers = tuple(
            node.create_publisher(message_type, topic, qos)
            for topic in self.topics
        )

    def publish(self, message):
        for publisher in self.publishers:
            publisher.publish(message)


class PreferredSourceGate:
    """Accept the highest-priority source that has been active recently."""

    def __init__(self, prioritized_topics, stale_after_s=1.0):
        self._priority = {
            topic: index for index, topic in enumerate(prioritized_topics)
        }
        self._stale_after_s = float(stale_after_s)
        self._last_seen = {}
        self._lock = threading.Lock()

    def accepts(self, topic):
        now = time.monotonic()
        with self._lock:
            self._last_seen[topic] = now
            active = [
                source for source, last_seen in self._last_seen.items()
                if now - last_seen <= self._stale_after_s
            ]
            selected = min(active, key=self._priority.__getitem__)
            return topic == selected


def create_priority_subscriptions(
    node, sources, callback, qos=10, stale_after_s=1.0,
):
    """Subscribe to ``(message_type, topic)`` pairs with source priority.

    ``sources`` must be ordered from preferred to fallback. The callback gets
    both the message and the topic that supplied it.
    """
    topics = [topic for _, topic in sources]
    gate = PreferredSourceGate(topics, stale_after_s=stale_after_s)
    subscriptions = []

    for message_type, topic in sources:
        def receive(message, source=topic):
            if gate.accepts(source):
                callback(message, source)

        subscriptions.append(
            node.create_subscription(message_type, topic, receive, qos)
        )
    return subscriptions


def first_ready_service_client(clients, wait_timeout_s=0.2):
    """Return the first ready client, preserving the caller's priority order."""
    clients = tuple(clients)
    for client in clients:
        if client.service_is_ready():
            return client
    for client in clients:
        if client.wait_for_service(timeout_sec=wait_timeout_s):
            return client
    return None
