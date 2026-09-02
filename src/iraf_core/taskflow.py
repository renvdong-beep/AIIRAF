from .core import TaskStatus

class InvalidTransition(RuntimeError):
    pass

TERMINAL = {TaskStatus.SUCCEEDED, TaskStatus.FAILED, TaskStatus.ABORTED, TaskStatus.CANCELLED, TaskStatus.SAFETY_STOP}
ALLOWED = {
    TaskStatus.PENDING: {TaskStatus.VALIDATING, TaskStatus.CANCELLED},
    TaskStatus.VALIDATING: {TaskStatus.RUNNING, TaskStatus.FAILED, TaskStatus.CANCELLED},
    TaskStatus.RUNNING: {TaskStatus.SUCCEEDED, TaskStatus.FAILED, TaskStatus.ABORTED, TaskStatus.CANCELLED, TaskStatus.SAFETY_STOP},
}

class TaskFlow:
    def __init__(self, execution_id):
        self.execution_id = execution_id
        self.status = TaskStatus.PENDING
        self.sequence = 0

    def transition(self, status, reason=''):
        if self.status in TERMINAL or status not in ALLOWED.get(self.status, set()):
            raise InvalidTransition(f'{self.status.value} -> {status.value}')
        self.status = status
        self.sequence += 1
        return {'execution_id': self.execution_id, 'sequence': self.sequence,
                'status': status.value, 'reason': reason}
