"""AgentOS task adapter；不承载 IRAF 领域执行逻辑。"""
class TaskDispatcher:
    def __init__(self,runtime): self.runtime=runtime
    @property
    def profile(self): return self.runtime.profile
    @property
    def safety_policy(self): return self.runtime.safety_policy
    @property
    def registry(self): return self.runtime.registry
    @property
    def resource_id(self): return self.runtime.resource_id
    def dispatch(self,payload,context): return self.runtime.execute(payload,context)
    def get(self,execution_id): return self.runtime.get(execution_id)