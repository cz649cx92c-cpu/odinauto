# Odin Goal UI

Lightweight local web UI for publishing `PoseStamped` goals in the current Odin
odometry frame. The server refuses goals while `/odin1/odometry` is stale.

```bash
./run.sh
```

Open `http://192.168.101.212:8090/`.

The stop action publishes `true` on `/sru/cancel_navigation`. The patched SRU
navigation node clears its active goal, resets model state, and immediately
publishes a zero velocity command.
