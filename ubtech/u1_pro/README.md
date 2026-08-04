# UBTECH U1 Pro driver adapter

The public capability layer is complete and stable, but no public, buildable U1
Pro ROS2 IDL/SDK was available. Commands therefore use a correlated JSON bridge:

```json
{"id":"uuid","command":"speech.say","params":{"text":"hello"},"timestamp":0}
```

Publish supplier state and acknowledgements on the configured `state` and `ack`
topics. The driver accurately reports `awaiting_supplier_ack`; it never reports
physical success without an acknowledgement. Replace `JsonCommandBridge` with a
typed supplier binding when the U1 Pro SDK and IDL package are delivered—the MCP
tools and their schemas do not need to change.
