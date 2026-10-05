"""Bounded action ownership, including late goal acknowledgements on failure.

Cleanup uses an executor pump independent of the mission's expired deadline.
A cancellation acknowledgement alone never proves that motion has terminated.
"""
import time


class ActionTerminationError(RuntimeError):
    """The client cannot establish that its dispatched action has terminated."""


class ManagedAction:
    TERMINAL = (4,5,6)  # ROS GoalStatus: succeeded, canceled, aborted.

    def __init__(self,name,sent,wait,pump,alive,record,timeout=5.,clock=time.monotonic,before_cleanup=lambda:None):
        self.name,self.sent,self.wait = name,sent,wait
        self.pump,self.alive,self.record = pump,alive,record
        self.timeout,self.clock = timeout,clock
        self.before_cleanup = before_cleanup
        self.handle = self.result = None

    def __enter__(self):
        try:
            self.handle = self.wait(self.sent)
            if not self.handle.accepted:
                raise RuntimeError(f'{self.name} goal rejected')
            self.result = self.handle.get_result_async()
            return self
        except BaseException:
            self.terminate()
            raise

    def __exit__(self,kind,error,traceback):
        self.terminate()
        return False

    def _until(self,future,deadline):
        while not future.done():
            remaining = deadline-self.clock()
            if remaining <= 0. or not self.alive():
                raise ActionTerminationError(f'{self.name} terminal state unknown')
            self.pump(min(.05,remaining))
        return future.result()

    def terminate(self):
        deadline = self.clock()+self.timeout
        try:
            self.before_cleanup()
            if self.handle is None:
                # A mission timeout can race a server accepting the goal.
                self.handle = self._until(self.sent,deadline)
            if not self.handle.accepted:
                return
            if self.result is None:
                self.result = self.handle.get_result_async()
            if not self.result.done():
                cancel = self.handle.cancel_goal_async()
                # Do not block on the cancellation response: even a rejected or
                # lost response can race a valid terminal action result.
                self._until(self.result,deadline)
                if cancel.done():
                    try:
                        response = cancel.result()
                        self.record(action=self.name,cancel_return_code=response.return_code)
                    except Exception as exc:
                        self.record(action=self.name,cancel_response_error=str(exc))
            wrapped = self.result.result()
            if wrapped is None or wrapped.status not in self.TERMINAL:
                raise ActionTerminationError(f'{self.name} returned nonterminal result')
            self.record(action=self.name,terminal_status=wrapped.status)
        except Exception as exc:
            self.record(action=self.name,termination_error=str(exc))
            if isinstance(exc,ActionTerminationError):
                raise
            raise ActionTerminationError(f'{self.name} termination failed: {exc}') from exc
