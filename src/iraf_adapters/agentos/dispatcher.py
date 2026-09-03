"""AgentOS task adapter; domain execution remains in iraf_core.runtime."""

class TaskDispatcher:
    def __init__(self, runtime):
        self.runtime = runtime

    @property
    def profile(self):
        return self.runtime.profile

    @property
    def safety_policy(self):
        return self.runtime.safety_policy

    @property
    def registry(self):
        return self.runtime.registry

    @property
    def resource_id(self):
        return self.runtime.resource_id

    def dispatch(self, payload, context):
        return self.runtime.execute(payload, context)

    def get(self, execution_id):
        return self.runtime.get(execution_id)

    def record_pre_dispatch_failure(self, payload, context, code, reason, metadata):
        return self.runtime.record_pre_dispatch_failure(
            payload, context, code, reason, metadata
        )

    def annotate(self, result, metadata):
        persisted = self.runtime.annotate_execution(result["execution_id"], metadata)
        if persisted is not None:
            return persisted
        result.update(metadata)
        return result
