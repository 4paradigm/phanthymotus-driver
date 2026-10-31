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
            "description": "processor",
            "multiInstance": False,
            "description": "A plugin that echoes back the input text.",
            "inputschema": {
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
                "x_action-params": {
                    "run":{

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

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            self.start()
            return {"state": "Plugin started."}
        if action == "stop":
            self.stop()
            return {"state": "Plugin stopped."}
        if action == "info":
            return {
                "state": "Plugin started." if self.enabled else "Plugin stopped.",
                "hardware": "false",
                
                }
        if action == "run":
            if not self.enabled:
                return {
                "states": "falled",
                "error": "Plugin is not enabled. Please start the plugin before running actions."
                }

            if not text:
                return {
                    "states": "falled",
                    "error": "Missing required parameter: 'text'. Please provide the text to echo back."
                }
            return {
                "states": "success",
                "input":text,
                "output":f"收到：{text}"
            }
        return None
