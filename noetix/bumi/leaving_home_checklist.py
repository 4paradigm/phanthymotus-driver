"""Compare a packing list with the items the user reports carrying."""

import re


DEFAULT_REQUIRED_ITEMS = "钥匙、手机、雨伞"


def parse_items(text: str) -> list[str]:
    items = (item.strip() for item in re.split(r"[、，,\n]+", text))
    return list(dict.fromkeys(item for item in items if item))


def check_items(required_items: str, carried_items: str) -> dict:
    required = parse_items(required_items) or parse_items(DEFAULT_REQUIRED_ITEMS)
    carried = set(parse_items(carried_items))
    missing = [item for item in required if item not in carried]
    return {
        "missing_items": missing,
        "all_ready": not missing,
        "message": "还没带：" + "、".join(missing) if missing else "必带物品已带齐。",
    }


class LeavingHomeChecklistPlugin:
    PREFIX = "leaving_home_checklist"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        """Accept the bundle's standard plugin inputs for this stateless card."""

    def get_tool(self) -> dict:
        return {
            "name": self.PREFIX,
            "type": "processor",
            "multiInstance": False,
            "description": "出门物品清单。比对必带物品和手动填写的已带物品，返回缺失物品。",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["check"]},
                    "required_items": {
                        "type": "string",
                        "default": DEFAULT_REQUIRED_ITEMS,
                        "description": "必带物品，用顿号或逗号分隔。留空用钥匙、手机、雨伞。",
                    },
                    "carried_items": {
                        "type": "string",
                        "description": "已带物品，用顿号或逗号分隔。留空表示未带任何物品。",
                    },
                },
                "required": ["action"],
            },
        }

    def start(self) -> None:
        """The checklist has no background work to start."""

    def stop(self) -> None:
        """The checklist owns no resources to release."""

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "info":
            return {"state": "ready", "default_required_items": parse_items(DEFAULT_REQUIRED_ITEMS)}
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action != "check":
            return None

        required = args.get("required_items", "")
        carried = args.get("carried_items", "")
        if not isinstance(required, str) or not isinstance(carried, str):
            return {"state": "invalid_parameters", "message": "必带物品和已带物品都应填写为文本。"}
        return check_items(required, carried)


def main() -> None:
    required = input(f"必带物品（回车使用默认清单 {DEFAULT_REQUIRED_ITEMS}）：")
    carried = input("已带物品（用顿号或逗号分隔）：")
    print(check_items(required, carried)["message"])


if __name__ == "__main__":
    main()
