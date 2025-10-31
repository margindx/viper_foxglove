from foxglove2zmq import FoxgloveToZMQPushRelay, FoxgloveToZMQPubSubRelay
import asyncio

if __name__ == "__main__":
    """
    Example script to run a Foxglove to ZMQ relay.
    
    This script demonstrates how to set up a relay that connects to a Foxglove WebSocket server
    and relays messages to ZMQ clients. It can be configured to use either a PUSH or PUB/SUB model.
    """

    print("🚀 Starting Foxglove to ZMQ relay...")

    # --- Configuration ---
    # Choose which relay to run by uncommenting it.

    # PUSH/PULL Example: Sends all messages to any connected PULL client.
    relay = FoxgloveToZMQPushRelay(
        foxglove_address="ws://localhost:8765",
        zmq_address="tcp://localhost:6555",
        zmq_listen_address="tcp://localhost:6556",
        topic_blocklist=None,
        discovery_timeout=2.0,
        verbosity=2
    )

    # # PUB/SUB Example: Publishes messages on topics for SUB clients to filter.
    # relay = FoxgloveToZMQPubSubRelay(
    #     foxglove_address="ws://localhost:8765",
    #     zmq_address="tcp://*:5555",
    #     topic_blocklist=[
    #         "/some/topic/to/ignore",
    #         "/another/debug/topic",
    #     ],
    #     discovery_timeout=2.0
    # )

    try:
        asyncio.run(relay.run())
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user. Shutting down.")