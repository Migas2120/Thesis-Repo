# ros/control_center.py

class ControlCenter:
    def __init__(self, dispatcher, state_manager, logger, handlers: dict):
        """
        dispatcher: handles command forwarding (e.g., to services)
        state_manager: manages drone's internal state
        logger: optional logger
        handlers: dict mapping command strings to callable handler functions
        """
        self.dispatcher = dispatcher
        self.state = state_manager
        self.logger = logger
        self.handlers = handlers

    def process_message(self, data: dict):
        if not data:
            self._log("warning", "Received empty message.")
            return

        command = data.get("command") or data.get("intent")
        if not command:
            self._log("warning", f"Missing 'command' or 'intent' in: {data}")
            return

        handler = self.handlers.get(command)
        if not handler:
            self._log("warning", f"No handler for command '{command}'")
            return

        try:
            handler(data, self.dispatcher, self.state, self.logger)
        except Exception as e:
            self._log("error", f"Exception while handling '{command}': {e}")

    def _log(self, level, msg):
        tag = "[ControlCenter]"
        if self.logger:
            log_fn = getattr(self.logger, level, self.logger.info)
            log_fn(f"{tag} {msg}")
        else:
            print(f"{tag} {msg}")
