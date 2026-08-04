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

The robot has not arrived yet. This draft validates the MCP contract and the
correlation/acknowledgement behavior only; it does not claim that any U1 Pro
motion has executed. Supplier SDK/IDL, topic bindings, action IDs, limits and
hardware evidence are required before the adapter can be marked ready.
