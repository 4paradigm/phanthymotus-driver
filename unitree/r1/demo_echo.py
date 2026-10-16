from typing import Optional

class DemoEchoPlugin:

    PREFIX = "demo_echo"
    def __init__(
        self,
        plugin_config: dict,
        namespace: str,
        executor,
    ):

        self.enabled = True

    def get_tool(self) -> dict:
        return {
            "name": self.PREFIX,
            "type": "processor",
            "multiInstance": False,
            "description": "A plugin that echoes back the input text.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["run"],
                        "description": "The action to perform. Must be 'run'.",
                    },
                    "text": {
                        "type": "string",
                        "description": "The text to echo back.",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "run": {
                        "params": ["text"],
                        "description": "Echoes back the provided text.",
                    }
                }
            },
        }
    def start(self) -> None:
        self.enabled = True

    def stop(self) -> None:
        self.enabled = False

    def dispatch(self, action: str, args: dict) -> Optional[dict]:
        if action == "start":
            self.start()
            return {"state": "running"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "info":
            return {
                "state": "running" if self.enabled else "idle",
                "hardware": "false",
                }
        if action == "run":
            if not self.enabled:
                return {
                    "state": "error",
                    "error": "Plugin is not enabled. Please start the plugin before running actions."
                }

            text = args.get("text", "")
            if not text:
                return {
                    "state": "error",
                    "error": "Input text cannot be empty."
                }
            return {
                "state": "completed",
                "input": text,
                "output": f"received:{text}"
            }
        return None
